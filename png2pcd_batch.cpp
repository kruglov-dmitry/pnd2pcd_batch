/*
 *          png2pcd_batch   -   simple command line utility to convert depth and rgb frames
 *          from png format to PCL pointcloud.
 *
 *          There are exists 2 execution mode:
 *          using file with association information i.e. containing rgb-depth png files correspondence
 *          or just providing folders that contain depth and rgb frames ( not reccommended ).
 *
 *          In 1t case you should anyhow create associate file by yourself.
 *          In 2nd case - correspondence strictly depends on file names, and you should check it twice,
 *          to avoid situation when selected depth frame do not appropriate for rgb frame.
 *
 *          All dataset related parameters are incapsulated in Intr structure ( intrinsics ).
 *          There are: width, height, fx, fy, cx, cy, scale_factor.
 *          Usually depth data is saved as unsigned short ( 16 bit ), 
 *          but in pcl::PointXYZ you have to re-scale it to float - metric measurment.
 *
 *          Appropriate intrinsics should be written to cam_params.cfg file otherwise
 *          default values will be used ( which may lead to invalid output data ).
 *
 *          There are exist two opportunity for compiling:
 *
 *          using classical make:
 *          edit WORK_DIR in Makefile to point in directory contained pcl-trunk & opencv
 *          make
 *          it produce more lighweighted version by avoiding linkage with redundant libraries
 *
 *          or using cmake:
 *
 *          mkdir build; cd build
 *          cmake ..
 *          make
 *
 *          NOTE 1: There are only two dependencies:
 *          PCL and OpenCV.
 *
 *          NOTE 2: in case of builded-but-not-properly-installed OpenCV libraries you have to 
 *          manually create symlink to ipp lib:
 *          sudo ln -s /path-to-opencv/3rdparty/ippicv/unpack/ippicv_lnx/lib/intel64/libippicv.a /usr/lib/libippicv.a
 *
 *
 * */


#include <pcl/io/pcd_io.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> // imread

#include <boost/filesystem.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using namespace pcl;
using namespace std;

/*
 * represent camera intrinsics params
 * scale factor used for convert depth values from unsigned short to float
 * for kinect-based dataset - usually 0.001
 *
 * */ 

typedef struct Intr
{
    int width;
    int height;
    float fx;
    float fy;
    float cx;
    float cy;
    float scale_factor;
} Intr;

/*  Predefined data for freiburg dataset 3 
 *  */
const Intr DEFAULT_CAM_PARAMS   =   {

                                    640,
                                    480,
                                    535.4f,
                                    539.2f,
                                    320.1f,
                                    247.6f,
                                    ( 1.f / 5000.f )
                                };

const string DEFAULT_CFG_FILE   =   "cam_params.cfg";

void
usage ()
{
    cout    <<  "Usage: "   << endl
            <<  "1 case: png2pcd_batch association_file" << endl
            <<  "\t\t   where association_file is a text file which should be parsed by USER implemented rule" << endl
            <<  "\t\t   for details and example of implementation check \"parse_freiburg\" function in source" << endl
            <<  "2 case: png2pcd_batch folder_with_rgb folder_with_depth" << endl
            <<  "\t\t   1st folder should contain png files with RGB data () " << endl
            <<  "\t\t   2nd folder should contain png files with DEPTH data () " << endl;
    exit(0);
}

/*
 *          This is just an example of function for parsing associating file,
 *          which should point which rgb frame is a pair for particular depth frame.
 *          This particular function aimed for processing freiburg3 dataset
 *          where association file can be generated in a following way:
 *         
 *          download test data: 
 *          $ mkdir data; cd data
 *          $ wget http://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz
 *          $ tar xvzf rgbd_dataset_freiburg3_long_office_household.tgz
 *          $ cd ..
 *
 *          download tools for RGB-D benchmark
 *          $ svn checkout https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools
 *          $ cp rgbd_benchmark_tools/src/rgbd_benchmark_tools/associate.py .
 *
 *          run associate script in accordance with recommendation of dataset provider in order to generate associate file:
 *          $ ./associate.py ~/data/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt ~/data/rgbd_dataset_freiburg3_long_office_household/depth.txt > tmp.txt
 *          $ ./associate.py tmp.txt ~/data/rgbd_dataset_freiburg3_long_office_household/rgb.txt > ~/data/rgbd_dataset_freiburg3_long_office_household/associate.txt
 *
 *          NOTE: almost every database have its own camera parameters: fx,fy,cx,cy,scale_factor - do not forget load appropriate
 *          NOTE 2: for automation of load such intrinsics params you can utilise load_camera_intrinsics function applying it to
 *          file named cam_params.cfg
 */
bool
parse_freiburg ( string & file_name, vector<string> & depth_names, vector<string> & rgb_names, vector <string> & pcd_file_names )
{
    
    fstream assoc_file; 
    
    // used during stream parsing only to skip unuseful data
    float time_stamp;
    float q1, q2, q3, q4,   /* quaternion fractions */
    tr1, tr2, tr3;          /* translation part */
    
    string  depth_name; 
    string  rgb_name;

    assoc_file.open ( file_name.c_str(), std::ios::in );
    if ( ! assoc_file.is_open() )
    {
        cerr << "Can't read association file: " << file_name << endl;
        return false;
    }
    else
    {
        while( ! assoc_file.eof())
        {

            string cur_line ("");
            getline ( assoc_file, cur_line );
            stringstream magic_stream ( cur_line );

            // skip all unuseful data - according to format of particular file
            magic_stream >> time_stamp;
            magic_stream >> tr1 >> tr2 >> tr3;
            magic_stream >> q1 >> q2 >> q3 >> q4;
            magic_stream >> time_stamp;
            
            magic_stream >> depth_name;
            if( ! cur_line.empty() )
            {
                depth_names.push_back ( depth_name );
                magic_stream >> time_stamp; 
                magic_stream >> rgb_name; 
                rgb_names.push_back ( rgb_name );
                
                char tmp_str[128];
                memset ( tmp_str, 0, 128 );
                sprintf ( tmp_str, "%05d.pcd", pcd_file_names.size() );

                pcd_file_names.push_back(tmp_str);
            }
        }
    }

    cout << "Total parsed "<< pcd_file_names.size()<< " files"<< endl;	
    return true;
}

bool
load_camera_intrinsics ( const string & cam_param_file_name, Intr & param )
{
    fstream cam_param; 
    cam_param.open ( cam_param_file_name.c_str(), std::ios::in );
    if ( !cam_param.is_open() ) {
        cerr << "NOTE:  If you want to use custom camera params - create file named "<< endl
             << "\t\t"<< DEFAULT_CFG_FILE << "and add there values of width height fx fy cx cy scale_factor" << endl
             << "\t\t"<< " for your dataset. Whitespace as separator." << endl
             << "\t\t"<< " Every depth measurment is multiplied by scale_factor." << endl
             << "NOTE: Load default values - instrinsics for freiburg dataset!" << endl;
        param = DEFAULT_CAM_PARAMS;
        return false;
    }
    else
        cam_param >> param.width >> param.height >> param.fx >> param.fy >> param.cx >> param.cy >> param.scale_factor;

    return true;
}

void
load_names_of_all_files_from_dir ( const char * dir_name, vector < string > & vector_with_names )
{
    const boost::filesystem::path base_dir ( dir_name );
    string extension (".png");
    
    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
    {      
        boost::filesystem::path p = it->path ();
        if ( boost::filesystem::is_regular_file ( it->status () ) && boost::filesystem::extension ( it->path () ) == extension )
            vector_with_names.push_back ( it->path().string() );
    }
}

template < class T >
void
set_pixel ( T & pcl_pixel, cv::Mat & src, int x, int y, Intr& cam_params )
{
    cerr << "set_pixel: Error - do not have proper specification for type: " << typeid(T).name() << endl;
    throw;
}

template <>
void
set_pixel ( RGB & pcl_color_pixel, cv::Mat & src, int x, int y, Intr& cam_params )
{
    uint32_t rgb;
    cv::Vec3b cur_rgb = src.at<cv::Vec3b>(y,x);// b,g,r
    rgb =   ( static_cast<int> ( cur_rgb [ 2 ] ) ) << 16 |
            ( static_cast<int> ( cur_rgb [ 1 ] ) ) << 8 |
            ( static_cast<int> ( cur_rgb [ 0 ] ) );
    
    pcl_color_pixel.rgba = static_cast < uint32_t > ( rgb );
}

template <>
void
set_pixel ( pcl::PointXYZ & xyz_pcl_pixel, cv::Mat & src, int x, int y, Intr& cam_params )
{
    xyz_pcl_pixel.z = src.at<unsigned short>( y * cam_params.width + x ) * cam_params.scale_factor;
    xyz_pcl_pixel.x = xyz_pcl_pixel.z * ( x - cam_params.cx ) / cam_params.fx;
    xyz_pcl_pixel.y = xyz_pcl_pixel.z * ( y - cam_params.cy ) / cam_params.fy;
}

template < class T>
bool
load_cloud ( const string & file_name, PointCloud<T> & pcl_cloud , Intr& cam_params )
{

     cv::Mat cur_mat =   cv::imread ( file_name.c_str(), -1 );
	 cv::Size s      =   cur_mat.size();
	 int width       =   s.width;
	 int height      =   s.height;
     int nchannels   =   cur_mat.channels();
     int step        =   cur_mat.step;

     pcl_cloud.width   = width;
     pcl_cloud.height  = height;
     pcl_cloud.is_dense = true;
     pcl_cloud.points.resize ( width * height );

     for ( int y = 0; y < height; y++ )
        for ( int x = 0; x < width; x++ )
        {
            T   current_pixel;

            set_pixel <T> ( current_pixel, cur_mat, x, y, cam_params );
                  
            pcl_cloud (x, y ) = current_pixel;

        }

}

int
main ( int argc, char* argv[] )
{
  
    vector < string > depth_names;
    vector < string > rgb_names;
    vector < string > pcd_file_names;
    
    string assoc_file = argv[1];

    if ( argc == 2 ) // it mean user pass only associated file
        parse_freiburg ( assoc_file, depth_names, rgb_names, pcd_file_names );
    else
    if ( argc == 3 )
    {
        load_names_of_all_files_from_dir ( argv[1], depth_names );
        load_names_of_all_files_from_dir ( argv[2], rgb_names );
    }

    // do we have data for continue ?
    if ( depth_names.empty() || depth_names.size() != rgb_names.size() )
        usage();

    Intr cam_params; 
    load_camera_intrinsics ( DEFAULT_CFG_FILE, cam_params );

    for ( int i = 0; i < pcd_file_names.size(); i++)
    {
        PointCloud < RGB >          current_color_cloud;
        PointCloud < PointXYZ >     current_xyz_cloud;
        PointCloud < PointXYZRGBA > current_xyzrgb_cloud;

        load_cloud < RGB > ( rgb_names[i], current_color_cloud, cam_params );
        
        load_cloud < PointXYZ > ( depth_names[i], current_xyz_cloud, cam_params );

        copyPointCloud ( current_xyz_cloud, current_xyzrgb_cloud );
        for ( size_t ii = 0; ii < current_color_cloud.size (); ++ii )
            current_xyzrgb_cloud.points[ii].rgba = current_color_cloud.points[ii].rgba;
                
        pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZRGBA> ( pcd_file_names[i], current_xyzrgb_cloud );
    }
    return 0;
}




