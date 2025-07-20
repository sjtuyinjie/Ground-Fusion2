#include "voxel_mapping.hpp"
#include "meshing/mesh_rec_display.hpp"
#include "meshing/mesh_rec_geometry.hpp"
#include "tools/tools_thread_pool.hpp"
#include <pcl/filters/voxel_grid.h>

extern Global_map       g_map_rgb_pts_mesh;
extern Triangle_manager g_triangles_manager;
extern int              g_current_frame;

extern double                       minimum_pts;
extern double                       g_meshing_voxel_size;
extern FILE *                       g_fp_cost_time;
extern FILE *                       g_fp_lio_state;
extern bool                         g_flag_pause;
extern const int                    number_of_frame;
extern int                          appending_pts_frame;
extern LiDAR_frame_pts_and_pose_vec g_eigen_vec_vec;

int        g_maximum_thread_for_rec_mesh;
std::mutex g_mutex_append_map;
std::mutex g_mutex_reconstruct_mesh;

extern double g_LiDAR_frame_start_time;
double        g_vx_map_frame_cost_time;
static double g_LiDAR_frame_avg_time;

namespace{
    const double image_obs_cov = 1.5;
    const double process_noise_sigma = 0.15;
}

namespace {
    std::unique_ptr<std::thread> g_pub_thr = nullptr;
    int flag = 0;
}

struct Rec_mesh_data_package
{
    pcl::PointCloud< pcl::PointXYZI >::Ptr m_frame_pts;
    Eigen::Quaterniond                     m_pose_q;
    Eigen::Vector3d                        m_pose_t;
    int                                    m_frame_idx;
    Rec_mesh_data_package( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts, Eigen::Quaterniond pose_q, Eigen::Vector3d pose_t, int frame_idx )
    {
        m_frame_pts = frame_pts;
        m_pose_q = pose_q;
        m_pose_t = pose_t;
        m_frame_idx = frame_idx;
    }
};

std::mutex                                  g_mutex_data_package_lock;
std::list< Rec_mesh_data_package >          g_rec_mesh_data_package_list;
std::shared_ptr< Common_tools::ThreadPool > g_thread_pool_rec_mesh = nullptr;

extern int                                  g_enable_mesh_rec;
extern int                                  g_save_to_offline_bin;

LiDAR_frame_pts_and_pose_vec                                                                               g_ponintcloud_pose_vec;

void transformPointCloudToFusionFrame(
    pcl::PointCloud<pcl::PointXYZI>::Ptr frame_pts,
    Eigen::Vector3d orin_pose_t,
    Eigen::Quaterniond orin_pose_q,
    Eigen::Vector3d pose_t,
    Eigen::Quaterniond pose_q,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& transformed_pts,
    Eigen::Matrix3d& R_imu_to_fusion,
    Eigen::Vector3d& t_imu_to_fusion)
{
    Eigen::Matrix3d R_imu_to_world = orin_pose_q.toRotationMatrix();
    Eigen::Vector3d t_imu_to_world = orin_pose_t;

    Eigen::Matrix3d R_fusion_to_world = pose_q.toRotationMatrix();
    Eigen::Vector3d t_fusion_to_world = pose_t;

    // Eigen::Matrix3d R_imu_to_fusion = R_fusion_to_world.transpose() * R_imu_to_world;
    // Eigen::Vector3d t_imu_to_fusion = R_fusion_to_world.transpose() * (t_imu_to_world - t_fusion_to_world);
    
    R_imu_to_fusion = R_imu_to_world * R_fusion_to_world.transpose();
    t_imu_to_fusion = R_imu_to_fusion * t_imu_to_world - t_fusion_to_world;

    // std::cout << "R_imu_to_fusion:\n" << R_imu_to_fusion << std::endl;
    // std::cout << "t_imu_to_fusion:\n" << t_imu_to_fusion.transpose() << std::endl;

    transformed_pts->clear();
    for (const auto& point : frame_pts->points) {
        Eigen::Vector3d p_imu(point.x, point.y, point.z);
        Eigen::Vector3d p_fusion =  R_imu_to_fusion * p_imu - t_imu_to_fusion;

        pcl::PointXYZI transformed_point;
        transformed_point.x = p_fusion.x();
        transformed_point.y = p_fusion.y();
        transformed_point.z = p_fusion.z();
        transformed_point.intensity = point.intensity;

        transformed_pts->points.push_back(transformed_point);
    }
}

void incremental_mesh_reconstruction( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts, cv::Mat img, Eigen::Quaterniond pose_q, Eigen::Vector3d pose_t, int frame_idx, Eigen::Vector3d orin_pose_t, Eigen::Quaterniond orin_pose_q )
{
    while ( g_flag_pause )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
    }

    Eigen::Matrix3d R_imu_to_fusion;
    Eigen::Vector3d t_imu_to_fusion;
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pts(new pcl::PointCloud<pcl::PointXYZI>);
    transformPointCloudToFusionFrame(frame_pts, orin_pose_t, orin_pose_q, pose_t, pose_q, transformed_pts, R_imu_to_fusion, t_imu_to_fusion);
    frame_pts = transformed_pts;

    Eigen::Matrix< double, 7, 1 > pose_vec;
    pose_vec.head< 4 >() = pose_q.coeffs().transpose();
    pose_vec.block( 4, 0, 3, 1 ) = pose_t;
    for ( int i = 0; i < frame_pts->points.size(); i++ )
    {
        g_eigen_vec_vec[ frame_idx ].first.emplace_back( frame_pts->points[ i ].x, frame_pts->points[ i ].y, frame_pts->points[ i ].z,
                                                         frame_pts->points[ i ].intensity );
    }
    g_eigen_vec_vec[ frame_idx ].second = pose_vec;
    // g_eigen_vec_vec.push_back( std::make_pair( empty_vec, pose_vec ) );
    // TODO : add time tic toc

    int                 append_point_step = std::max( ( int ) 1, ( int ) std::round( frame_pts->points.size() / appending_pts_frame ) );
    Common_tools::Timer tim, tim_total, tim_append, time_mesh, tim_render;

    tim_append.tic();
    int acc = 0;
    int rej = 0;
    std::unordered_set< std::shared_ptr< RGB_Voxel > > voxels_recent_visited;
    voxels_recent_visited.clear();

    int pt_size = frame_pts->points.size();
    KDtree_pt_vector     pt_vec_vec;
    std::vector< float > dist_vec;

    RGB_voxel_ptr* temp_box_ptr_ptr;
    double minimum_pts_size = g_map_rgb_pts_mesh.m_minimum_pts_size;
    double voxel_resolution = g_map_rgb_pts_mesh.m_voxel_resolution;

    g_mutex_append_map.lock();
    //存入数据到网格和体素
    for ( long pt_idx = 0; pt_idx < pt_size; pt_idx += append_point_step )
    {
        int  add = 1;
        int  grid_x = std::round( frame_pts->points[ pt_idx ].x / minimum_pts_size ); //存入网格
        int  grid_y = std::round( frame_pts->points[ pt_idx ].y / minimum_pts_size );
        int  grid_z = std::round( frame_pts->points[ pt_idx ].z / minimum_pts_size );
        int  box_x = std::round( frame_pts->points[ pt_idx ].x / voxel_resolution );
        int  box_y = std::round( frame_pts->points[ pt_idx ].y / voxel_resolution );
        int  box_z = std::round( frame_pts->points[ pt_idx ].z / voxel_resolution );
        auto pt_ptr = g_map_rgb_pts_mesh.m_hashmap_3d_pts.get_data( grid_x, grid_y, grid_z );
        if ( pt_ptr != nullptr )
            add = 0;

        RGB_voxel_ptr box_ptr;
        temp_box_ptr_ptr = g_map_rgb_pts_mesh.m_hashmap_voxels.get_data( box_x, box_y, box_z );
        if ( temp_box_ptr_ptr == nullptr )
        {
            box_ptr = std::make_shared< RGB_Voxel >( box_x, box_y, box_z );
            g_map_rgb_pts_mesh.m_hashmap_voxels.insert( box_x, box_y, box_z, box_ptr );
//            g_map_rgb_pts_mesh.m_voxel_vec.push_back( box_ptr );
        }
        else
        {
            box_ptr = *temp_box_ptr_ptr;
        }
        voxels_recent_visited.insert( box_ptr );
        box_ptr->m_last_visited_time = frame_idx;
        if ( add == 0 )
        {
            rej++;
            continue;
        }

        acc++;
        KDtree_pt kdtree_pt( vec_3( frame_pts->points[ pt_idx ].x, frame_pts->points[ pt_idx ].y, frame_pts->points[ pt_idx ].z ), 0 );
        if ( g_map_rgb_pts_mesh.m_kdtree.Root_Node != nullptr )
        {
            g_map_rgb_pts_mesh.m_kdtree.Nearest_Search( kdtree_pt, 1, pt_vec_vec, dist_vec );
            if ( pt_vec_vec.size() )
            {
                if ( sqrt( dist_vec[ 0 ] ) < minimum_pts_size )
                {
                    continue;
                }
            }
        }

        std::shared_ptr< RGB_pts > pt_rgb = std::make_shared< RGB_pts >();

        double x = frame_pts->points[pt_idx].x;
        double y = frame_pts->points[pt_idx].y;
        double z = frame_pts->points[pt_idx].z;
        pt_rgb->set_pos(vec_3(x, y, z));

        pt_rgb->m_pt_index = g_map_rgb_pts_mesh.m_rgb_pts_vec.size();
        kdtree_pt.m_pt_idx = pt_rgb->m_pt_index;
        g_map_rgb_pts_mesh.m_rgb_pts_vec.push_back( pt_rgb );

        g_map_rgb_pts_mesh.m_hashmap_3d_pts.insert( grid_x, grid_y, grid_z, pt_rgb ); //存入RGB点和坐标
        if ( box_ptr != nullptr )
        {
            box_ptr->m_pts_in_grid.push_back( pt_rgb );
            // box_ptr->add_pt(pt_rgb);
            box_ptr->m_new_added_pts_count++;
            box_ptr->m_meshing_times = 0;
//            file << "new points | " << "use_count " << box_ptr.use_count() << " | m_pts_in_grid.size() " << box_ptr->m_pts_in_grid.size() << endl;
        }
        else
        {
            scope_color( ANSI_COLOR_RED_BOLD );
            for ( int i = 0; i < 100; i++ )
            {
                cout << "box_ptr is nullptr!!!" << endl;
            }
        }
        // Add to kdtree
        g_map_rgb_pts_mesh.m_kdtree.Add_Point( kdtree_pt, true );

    }
    g_mutex_append_map.unlock();

    // std::cout<< "frame_idx: " << frame_idx <<  "append_points_to_global_map" << std::endl;

    g_map_rgb_pts_mesh.m_mutex_m_box_recent_hitted->lock();
    g_map_rgb_pts_mesh.m_voxels_recent_visited = voxels_recent_visited;
    g_map_rgb_pts_mesh.m_mutex_m_box_recent_hitted->unlock();

    double time_a = tim_append.toc();

    Eigen::Matrix3d rot_i2w = orin_pose_q.toRotationMatrix();
    Eigen::Vector3d pos_i2w = orin_pose_t;

    Eigen::Matrix3d R_c2w;
    Eigen::Vector3d t_c2w;

    Eigen::Matrix3d R_c2i;
    Eigen::Vector3d t_c2i;

    Eigen::Matrix3d R_w2c;
    Eigen::Vector3d t_w2c;

    R_c2i = extR;
    t_c2i = extT;

    R_c2w = R_c2i * rot_i2w;
    t_c2w = rot_i2w * t_c2i + pos_i2w;
    
    R_w2c = R_c2w * R_imu_to_fusion;
    t_w2c = R_imu_to_fusion * t_c2w - t_imu_to_fusion;

    std::shared_ptr<Image_frame> image_pose = std::make_shared<Image_frame>(cam_k);
    image_pose->set_pose(eigen_q(R_w2c), t_w2c);
    image_pose->m_img = img;
    image_pose->m_timestamp = ros::Time::now().toSec();
    image_pose->init_cubic_interpolation();
    image_pose->image_equalize();

    tim_render.tic();
    auto numbers_of_voxels = voxels_recent_visited.size();
    std::vector<shared_ptr<RGB_Voxel>> voxels_for_render;
    for ( Voxel_set_iterator it = voxels_recent_visited.begin(); it != voxels_recent_visited.end(); it++ )
    {
        voxels_for_render.push_back( *it );
    }

    image_pose->m_acc_render_count = 0;
    image_pose->m_acc_photometric_error = 0;
    try
    {
        cv::parallel_for_(cv::Range(0, numbers_of_voxels), [&](const cv::Range &r) {
            vec_3 pt_w;
            vec_3 rgb_color;
            double u, v;
            double pt_cam_norm;
            g_mutex_append_map.lock();

            for (int voxel_idx = r.start; voxel_idx < r.end; voxel_idx++)
            {
                RGB_voxel_ptr voxel_ptr = voxels_for_render[voxel_idx];
                for (int pt_idx = 0; pt_idx < voxel_ptr->m_pts_in_grid.size(); pt_idx++)
                {
                    pt_w = voxel_ptr->m_pts_in_grid[pt_idx]->get_pos();
                    if (image_pose->project_3d_point_in_this_img(pt_w, u, v, nullptr, 1.0) == false)
                        continue;

                    pt_cam_norm = (pt_w - image_pose->m_pose_w2c_t).norm();
                    rgb_color = image_pose->get_rgb(u, v, 0);

                    if (voxel_ptr->m_pts_in_grid[pt_idx]->update_rgb(
                            rgb_color, pt_cam_norm, vec_3(image_obs_cov, image_obs_cov, image_obs_cov),
                            image_pose->m_timestamp)) {
//                        my_render_pts_count++;
                    }

                }
            }

            g_mutex_append_map.unlock();
        });
    }
    catch ( ... )
    {
        for ( int i = 0; i < 100; i++ )
        {
            cout << ANSI_COLOR_RED_BOLD << "Exception in tbb parallels...in rendering" << ANSI_COLOR_RESET << endl;
        }
        return;
    }
    double time_b = tim_render.toc();

    std::atomic< int >    voxel_idx( 0 );

    std::mutex mtx_triangle_lock, mtx_single_thr;
    typedef std::unordered_set< std::shared_ptr< RGB_Voxel > >::iterator set_voxel_it;
    std::unordered_map< std::shared_ptr< RGB_Voxel >, Triangle_set >     removed_triangle_list;
    std::unordered_map< std::shared_ptr< RGB_Voxel >, Triangle_set >     added_triangle_list;
    g_mutex_reconstruct_mesh.lock();


    tim.tic();
    tim_total.tic();


    time_mesh.tic();
    try
    {
        tbb::parallel_for_each( voxels_recent_visited.begin(), voxels_recent_visited.end(), [ & ]( const std::shared_ptr< RGB_Voxel > &voxel ) {
            // std::unique_lock<std::mutex> thr_lock(mtx_single_thr);
            // printf_line;
            if ( ( voxel->m_meshing_times >= 1 ) || ( voxel->m_new_added_pts_count < 0 ) )
            {
                return;
            }

            if (voxel->m_pts_in_grid.size() < 5) return;

            Common_tools::Timer tim_lock;
            tim_lock.tic();
            voxel->m_meshing_times++;
            voxel->m_new_added_pts_count = 0;
//            vec_3 pos_1 = vec_3( voxel->m_pos[ 0 ], voxel->m_pos[ 1 ], voxel->m_pos[ 2 ] );

            // printf("Voxels [%d], (%d, %d, %d) ", count, pos_1(0), pos_1(1), pos_1(2) );
            std::unordered_set< std::shared_ptr< RGB_Voxel > > neighbor_voxels;
            neighbor_voxels.insert( voxel );
            g_mutex_append_map.lock();
            std::vector< RGB_pt_ptr > pts_in_voxels = retrieve_pts_in_voxels( neighbor_voxels );
            if ( pts_in_voxels.size() < 3 )
            {
                g_mutex_append_map.unlock();
                return;
            }
            g_mutex_append_map.unlock();
            // Voxel-wise mesh pull
            pts_in_voxels = retrieve_neighbor_pts_kdtree( pts_in_voxels );
            pts_in_voxels = remove_outlier_pts( pts_in_voxels, voxel );

            std::set< long > relative_point_indices;
            for ( RGB_pt_ptr tri_ptr : pts_in_voxels )
            {
                relative_point_indices.insert( tri_ptr->m_pt_index );
            }

            int iter_count = 0;
            g_triangles_manager.m_enable_map_edge_triangle = 0;

           pts_in_voxels.clear();
           for ( auto p : relative_point_indices )
           {
               pts_in_voxels.push_back( g_map_rgb_pts_mesh.m_rgb_pts_vec[ p ] );
           }
           
            std::set< long > convex_hull_index, inner_pts_index;
            // mtx_triangle_lock.lock();
            voxel->m_short_axis.setZero();
            std::vector< long > add_triangle_idx = delaunay_triangulation( pts_in_voxels, voxel->m_long_axis, voxel->m_mid_axis,
                                                                               voxel->m_short_axis, convex_hull_index, inner_pts_index );

        
            Triangle_set triangles_sets = g_triangles_manager.find_relative_triangulation_combination( relative_point_indices );
            Triangle_set triangles_to_remove, triangles_to_add, existing_triangle;

            triangle_compare( triangles_sets, add_triangle_idx, triangles_to_remove, triangles_to_add, &existing_triangle );
            
            // Refine normal index
            for ( auto triangle_ptr : triangles_to_add )
            {
                correct_triangle_index( triangle_ptr, g_eigen_vec_vec[ frame_idx ].second.block( 4, 0, 3, 1 ), voxel->m_short_axis );
            }
            for ( auto triangle_ptr : existing_triangle )
            {
                correct_triangle_index( triangle_ptr, g_eigen_vec_vec[ frame_idx ].second.block( 4, 0, 3, 1 ), voxel->m_short_axis );
            }

            std::unique_lock< std::mutex > lock( mtx_triangle_lock );
            
            removed_triangle_list.emplace( std::make_pair( voxel, triangles_to_remove ) );
            added_triangle_list.emplace( std::make_pair( voxel, triangles_to_add ) );
            
            voxel_idx++;
        } );
    }
    catch ( ... )
    {
        for ( int i = 0; i < 100; i++ )
        {
            cout << ANSI_COLOR_RED_BOLD << "Exception in tbb parallels..." << ANSI_COLOR_RESET << endl;
        }
        return;
    }


    double              mul_thr_cost_time = tim.toc( " ", 0 );
    Common_tools::Timer tim_triangle_cost;
    int                 total_delete_triangle = 0, total_add_triangle = 0;
    // Voxel-wise mesh push
    for ( auto &triangles_set : removed_triangle_list )
    {
        total_delete_triangle += triangles_set.second.size();
        g_triangles_manager.remove_triangle_list( triangles_set.second );
    }

    for ( auto &triangle_list : added_triangle_list )
    {
        Triangle_set triangle_idx = triangle_list.second;
        total_add_triangle += triangle_idx.size();
        for ( auto triangle_ptr : triangle_idx )
        {
            Triangle_ptr tri_ptr = g_triangles_manager.insert_triangle( triangle_ptr->m_tri_pts_id[ 0 ], triangle_ptr->m_tri_pts_id[ 1 ],
                                                                        triangle_ptr->m_tri_pts_id[ 2 ], 1 );
            tri_ptr->m_index_flip = triangle_ptr->m_index_flip;
        }
    }
    
    g_mutex_reconstruct_mesh.unlock();
    tim_total.toc();

    double time_c = time_mesh.toc();
    double time_total = time_a + time_b + time_c;
    // LOG(INFO) << "frame_idx: " << frame_idx << " Points: " << pt_size << " | " << time_a << " | " << time_b   << " | " << time_c << "| "<< time_total;
    // std::cout<< "frame_idx: " << frame_idx << "mesh reconstruction cost: " << tim_total.toc() << std::endl;

    if ( g_fp_cost_time )
    {
        if ( frame_idx > 0 )
            g_LiDAR_frame_avg_time = g_LiDAR_frame_avg_time * ( frame_idx - 1 ) / frame_idx + ( g_vx_map_frame_cost_time ) / frame_idx;
        fprintf( g_fp_cost_time, "%d %lf %d %lf %lf\r\n", frame_idx, tim.toc( " ", 0 ), ( int ) voxel_idx.load(), g_vx_map_frame_cost_time,
                 g_LiDAR_frame_avg_time );
        fflush( g_fp_cost_time );
    }
    if ( g_current_frame < frame_idx )
    {
        g_current_frame = frame_idx;
    }
    else
    {
        if ( g_eigen_vec_vec[ g_current_frame + 1 ].second.size() > 7 )
        {
            g_current_frame++;
        }
    }
}

void service_reconstruct_mesh()
{
    if ( g_thread_pool_rec_mesh == nullptr )
    {
        g_thread_pool_rec_mesh = std::make_shared< Common_tools::ThreadPool >( g_maximum_thread_for_rec_mesh );
        // LOG(INFO) << "g_maximum_thread_for_rec_mesh: " << g_maximum_thread_for_rec_mesh;
    }
    int drop_frame_num = 0;
    while ( 1 )
    {
        
        while ( g_rec_color_data_package_list.size() == 0 )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        }

        g_mutex_all_data_package_lock.lock();
        auto data_pack_front = g_rec_color_data_package_list.front();
        g_rec_color_data_package_list.pop_front();
        if(data_pack_front.m_frame_pts == nullptr || data_pack_front.m_img.empty() || data_pack_front.m_frame_pts->points.size() == 0)
        {
            g_mutex_all_data_package_lock.unlock();
            continue;
        }

        g_mutex_all_data_package_lock.unlock();

        if ( g_enable_mesh_rec )
        {
            g_thread_pool_rec_mesh->commit_task( incremental_mesh_reconstruction, data_pack_front.m_frame_pts, data_pack_front.m_img ,data_pack_front.m_pose_q,
                                                 data_pack_front.m_pose_t, data_pack_front.m_frame_idx, data_pack_front.m_orin_position, data_pack_front.m_orin_rotation );
        }

        std::this_thread::sleep_for( std::chrono::microseconds( 5 ) );
    }
}
extern bool  g_flag_pause;
int          g_frame_idx = 0;
std::thread *g_rec_mesh_thr = nullptr;

void start_mesh_threads( int maximum_threads = 20 )
{
    if ( g_eigen_vec_vec.size() <= 0 )
    {
        g_eigen_vec_vec.resize( 1e6 );
    }
    if ( g_rec_mesh_thr == nullptr )
    {
        g_maximum_thread_for_rec_mesh = maximum_threads;
        g_rec_mesh_thr = new std::thread( service_reconstruct_mesh );
    }
}

void reconstruct_mesh_from_pointcloud( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts, double minimum_pts_distance )
{
    start_mesh_threads();
    cout << "=== reconstruct_mesh_from_pointcloud ===" << endl;
    cout << "Input pointcloud have " << frame_pts->points.size() << " points." << endl;
    pcl::PointCloud< pcl::PointXYZI >::Ptr all_cloud_ds( new pcl::PointCloud< pcl::PointXYZI > );

    pcl::VoxelGrid< pcl::PointXYZI > sor;
    sor.setInputCloud( frame_pts );
    sor.setLeafSize( minimum_pts_distance, minimum_pts_distance, minimum_pts_distance );
    sor.filter( *all_cloud_ds );

    cout << ANSI_COLOR_BLUE_BOLD << "Raw points number = " << frame_pts->points.size()
         << ", downsampled points number = " << all_cloud_ds->points.size() << ANSI_COLOR_RESET << endl;
    g_mutex_data_package_lock.lock();
    g_rec_mesh_data_package_list.emplace_back( all_cloud_ds, Eigen::Quaterniond::Identity(), vec_3::Zero(), 0 );
    g_mutex_data_package_lock.unlock();
}

void open_log_file()
{
    if ( g_fp_cost_time == nullptr || g_fp_lio_state == nullptr )
    {
        Common_tools::create_dir( std::string( Common_tools::get_home_folder() ).append( "/ImMesh_output" ).c_str() );
        std::string cost_time_log_name = std::string( Common_tools::get_home_folder() ).append( "/ImMesh_output/mesh_cost_time.log" );
        std::string lio_state_log_name = std::string( Common_tools::get_home_folder() ).append( "/ImMesh_output/lio_state.log" );
        // cout << ANSI_COLOR_BLUE_BOLD ;
        // cout << "Record cost time to log file:" << cost_time_log_name << endl;
        // cout << "Record LIO state to log file:" << cost_time_log_name << endl;
        // cout << ANSI_COLOR_RESET;
        g_fp_cost_time = fopen( cost_time_log_name.c_str(), "w+" );
        g_fp_lio_state = fopen( lio_state_log_name.c_str(), "w+" );
    }
}

std::vector< vec_4 > convert_pcl_pointcloud_to_vec( pcl::PointCloud< pcl::PointXYZI > &pointcloud )
{
    int                  pt_size = pointcloud.points.size();
    std::vector< vec_4 > eigen_pt_vec( pt_size );
    for ( int i = 0; i < pt_size; i++ )
    {
        eigen_pt_vec[ i ]( 0 ) = pointcloud.points[ i ].x;
        eigen_pt_vec[ i ]( 1 ) = pointcloud.points[ i ].y;
        eigen_pt_vec[ i ]( 2 ) = pointcloud.points[ i ].z;
        eigen_pt_vec[ i ]( 3 ) = pointcloud.points[ i ].intensity;
    }
    return eigen_pt_vec;
}

void Voxel_mapping::map_incremental_grow()
{
    // start_mesh_threads( m_meshing_maximum_thread_for_rec_mesh );
    if ( m_use_new_map )
    {
        while ( g_flag_pause )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
        }
        // startTime = clock();
        pcl::PointCloud< pcl::PointXYZI >::Ptr world_lidar( new pcl::PointCloud< pcl::PointXYZI > );
        pcl::PointCloud< pcl::PointXYZI >::Ptr world_lidar_full( new pcl::PointCloud< pcl::PointXYZI > );

        std::vector< Point_with_var > pv_list;
        // TODO: saving pointcloud to file
        // pcl::io::savePCDFileBinary(Common_tools::get_home_folder().append("/r3live_output/").append("last_frame.pcd").c_str(), *m_feats_down_body);
        transformLidar( state.rot_end, state.pos_end, m_feats_down_body, world_lidar );
        for ( size_t i = 0; i < world_lidar->size(); i++ )
        {
            Point_with_var pv;
            pv.m_point << world_lidar->points[ i ].x, world_lidar->points[ i ].y, world_lidar->points[ i ].z;
            M3D point_crossmat = m_cross_mat_list[ i ];
            M3D var = m_body_cov_list[ i ];
            var = ( state.rot_end * m_extR ) * var * ( state.rot_end * m_extR ).transpose() +
                  ( -point_crossmat ) * state.cov.block< 3, 3 >( 0, 0 ) * ( -point_crossmat ).transpose() + state.cov.block< 3, 3 >( 3, 3 );
            pv.m_var = var;
            pv_list.push_back( pv );
        }

        // pcl::PointCloud< pcl::PointXYZI >::Ptr world_lidar( new pcl::PointCloud< pcl::PointXYZI > );
        std::sort( pv_list.begin(), pv_list.end(), var_contrast );
        updateVoxelMap( pv_list, m_max_voxel_size, m_max_layer, m_layer_init_size, m_max_points_size, m_min_eigen_value, m_feat_map );
        double vx_map_cost_time = omp_get_wtime();
        g_vx_map_frame_cost_time = ( vx_map_cost_time - g_LiDAR_frame_start_time ) * 1000.0;
        // cout << "vx_map_cost_time = " <<  g_vx_map_frame_cost_time << " ms" << endl;

        open_log_file();
        if ( g_fp_lio_state != nullptr )
        {
            dump_lio_state_to_log( g_fp_lio_state );
        }
        g_frame_idx++;
    }

    if ( !m_use_new_map )
    {
        for ( int i = 0; i < m_feats_down_size; i++ )
        {
            /* transform to world frame */
            pointBodyToWorld( m_feats_down_body->points[ i ], m_feats_down_world->points[ i ] );
        }
        
        // add_to_offline_bin( state, m_Lidar_Measures.lidar_beg_time, m_feats_down_world );
        
#ifdef USE_ikdtree
#ifdef USE_ikdforest
        ikdforest.Add_Points( feats_down_world->points, lidar_end_time );
#else
        m_ikdtree.Add_Points( m_feats_down_world->points, true );
#endif
#endif
    }
}

inline void image_equalize(cv::Mat &img, int amp)
{
    cv::Mat img_temp;
    cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
    // Equalize gray image.
    clahe->apply(img, img_temp);
    img = img_temp;
}

inline cv::Mat equalize_color_image_Ycrcb(cv::Mat &image)
{
    cv::Mat hist_equalized_image;
    cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);

    //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
    std::vector<cv::Mat> vec_channels;
    cv::split(hist_equalized_image, vec_channels);

    //Equalize the histogram of only the Y channel
    // cv::equalizeHist(vec_channels[0], vec_channels[0]);
    image_equalize( vec_channels[0], 1 );
    cv::merge(vec_channels, hist_equalized_image);
    cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);
    return hist_equalized_image;
}

void Voxel_mapping::lidar_callback( const sensor_msgs::PointCloud2::ConstPtr &msg ) {
    std::lock_guard<std::mutex> lock(m_mutex_buffer); 

    double timestamp = msg->header.stamp.toSec();
    if (timestamp < m_last_timestamp_lidar)
    {
        ROS_ERROR("Lidar loop back detected, clearing buffer.");
        m_new_lidar_buffer.clear();
        // m_time_buffer.clear();
    }

    // PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud); // 直接转换，无需去畸变

    m_new_lidar_buffer.push_back(cloud);
    // m_time_buffer.push_back(timestamp);
    m_last_timestamp_lidar = timestamp;

    const size_t MAX_BUFFER_SIZE = 100;
    m_sig_buffer.notify_all();
}

void Voxel_mapping::image_callback(const sensor_msgs::CompressedImageConstPtr &msg) {

    m_mutex_buffer.lock();
    if(msg->header.stamp.toSec() < m_last_timestamp_img)
    {
        // LOG(ERROR) << "Image loop back, clear buffer";
        m_img_buffer.clear();
    }

    m_last_timestamp_img = msg->header.stamp.toSec();
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    double img_rec_time = msg->header.stamp.toSec();
    m_img = cv_ptr_compressed->image;
    cv_ptr_compressed->image.release();

    cv::remap( m_img, m_img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR );
    cv::cvtColor(m_img, m_img_gray, cv::COLOR_RGB2GRAY);

    image_equalize(m_img_gray, 3.0);
    m_img = equalize_color_image_Ycrcb(m_img);

    m_img_buffer.push_back(m_img);
    m_img_time_buffer.push_back(msg->header.stamp.toSec());

    //    LOG(INFO) << "[m_img_buffer] : " << m_img_buffer.size();
    //    LOG(INFO) << "[m_img_time_buffer] : " << m_img_time_buffer.size();

    m_mutex_buffer.unlock();
    m_sig_buffer.notify_all();
}
void Voxel_mapping::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(m_mutex_buffer); 

    double timestamp = msg->header.stamp.toSec();

    m_pose_buffer.push_back(*msg);

    m_sig_buffer.notify_all();
}

void Voxel_mapping::orin_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(m_mutex_buffer); 

    m_orin_pose_buffer.push_back(*msg);

    m_sig_buffer.notify_all();
}

void Voxel_mapping::sendData() {

    // LOG(INFO) << "Sending data";
    ros::Subscriber sub_li = m_ros_node_ptr->subscribe( "/scan", 200000, &Voxel_mapping::lidar_callback, this );
    ros::Subscriber sub_im = m_ros_node_ptr->subscribe( "/img", 200000, &Voxel_mapping::image_callback, this );
    ros::Subscriber sub_pose = m_ros_node_ptr->subscribe("/laser_pose", 200000, &Voxel_mapping::pose_callback, this);
    ros::Subscriber sub_orin_pose = m_ros_node_ptr->subscribe("/orin_laser_pose", 200000, &Voxel_mapping::orin_pose_callback, this);

    int data_id = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        // LOG(INFO) << "pose: " << m_pose_buffer.size() << "image: " << m_img_buffer.size() << "Lidar: " << m_new_lidar_buffer.size();
        start_mesh_threads( m_meshing_maximum_thread_for_rec_mesh);

        if (!m_orin_pose_buffer.empty() && !m_pose_buffer.empty() && !m_img_buffer.empty() && !m_new_lidar_buffer.empty()) {
            // LOG(INFO) << ": Sending data" << data_id;
            auto world_lidar_full = m_new_lidar_buffer.front();
            m_new_lidar_buffer.pop_front();


            auto img = m_img_buffer.front();
            m_img_buffer.pop_front();


            auto pose = m_pose_buffer.front();
            Eigen::Vector3d position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            Eigen::Quaterniond rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
            m_pose_buffer.pop_front();

            auto orin_pose = m_orin_pose_buffer.front();
            Eigen::Vector3d orin_position(orin_pose.pose.position.x, orin_pose.pose.position.y, orin_pose.pose.position.z);
            Eigen::Quaterniond orin_rotation(orin_pose.pose.orientation.w, orin_pose.pose.orientation.x, orin_pose.pose.orientation.y, orin_pose.pose.orientation.z);
            m_orin_pose_buffer.pop_front();

            if( !world_lidar_full->points.empty() && !img.empty() )
            {
                g_mutex_all_data_package_lock.lock();
                g_rec_color_data_package_list.emplace_back(world_lidar_full, img, rotation, position, data_id, orin_position, orin_rotation);
                g_mutex_all_data_package_lock.unlock();
                data_id++;
            }
        }
        else
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
            // if (m_pose_buffer.empty()) {
            //     LOG(INFO) << "Lack of pose data";
            // }
            // if (m_img_buffer.empty()) {
            //     LOG(INFO) << "Lack of img data";
            // }
            // if (m_new_lidar_buffer.empty()) {
            //     LOG(INFO) << "Lack of lidar data";
            // }
        }
    }
}