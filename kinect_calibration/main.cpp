
//------------------------------------------------------------------------------
//  2022/12/19作成
//
//
//  VC++ディレクトリ / インクルードディレクトリに
//      C:\opencv347\build\install\include
//      C:\Program Files\Azure Kinect SDK v1.3.0\sdk\include
//
//      C:\Program Files\Azure Kinect SDK v1.4.1\sdk\include        (自宅用 kinect sdk 1.4.1のとき使用)
//  を追加する．
//
//  VC++ディレクトリ / ライブラリディレクトリに
//      C:\opencv347\build\install\x64\vc16\lib
//      C:\Program Files\Azure Kinect SDK v1.3.0\sdk\windows-desktop\amd64\release\lib
//
//      C:\opencv347\build\install\lib      (自宅用 opencv347)
//      C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\lib          (自宅用 kinect sdk 1.4.1)
//
//------------------------------------------------------------------------------



// Visual C++でコンパイルするときにリンクするライブラリファイル
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui347.lib")
#pragma comment(lib, "opencv_calib3d347.lib")
#pragma comment(lib, "opencv_aruco347.lib")



#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <k4a/k4a.h>


#include "kinectdevice.h"

using namespace std;



//static string get_serial(k4a_device_t device)
//{
//    size_t serial_number_length = 0;
//
//    if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
//    {
//        cout << "Failed to get serial number length" << endl;
//        k4a_device_close(device);
//        exit(-1);
//    }
//
//    char* serial_number = new (std::nothrow) char[serial_number_length];
//    if (serial_number == NULL)
//    {
//        cout << "Failed to allocate memory for serial number (" << serial_number_length << " bytes)" << endl;
//        k4a_device_close(device);
//        exit(-1);
//    }
//
//    if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
//    {
//        cout << "Failed to get serial number" << endl;
//        delete[] serial_number;
//        serial_number = NULL;
//        k4a_device_close(device);
//        exit(-1);
//    }
//
//    string s(serial_number);
//    delete[] serial_number;
//    serial_number = NULL;
//    return s;
//}

static void print_calibration()
{



    uint32_t device_count = k4a_device_get_installed_count();
    cout << "Found " << device_count << " connected devices:" << endl;
    cout << fixed << setprecision(20);

    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {

        KinectDevice kinectdevice;
        //k4a_device_t device = NULL;
        //if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
        //{
        //    cout << deviceIndex << ": Failed to open device" << endl;
        //    exit(-1);
        //}

        k4a_calibration_t calibration;
        /*
        k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
        deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
        deviceConfig.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        deviceConfig.synchronized_images_only = true;
        */

        // get calibration
        if (K4A_RESULT_SUCCEEDED !=
            k4a_device_get_calibration(kinectdevice.device, kinectdevice.device_configuration.depth_mode, kinectdevice.device_configuration.color_resolution, &calibration))
        {
            cout << "Failed to get calibration" << endl;
            exit(-1);
        }

        k4a_calibration_camera_t calib = calibration.depth_camera_calibration;          // デプスカメラのキャリブレーションデータ
        //k4a_calibration_camera_t calib = calibration.color_camera_calibration;      // カラーカメラのキャリブレーション

        //cout << "\n===== Device " << (int)deviceIndex << ": " << get_serial(device) << " =====\n";
        cout << "resolution width: " << calib.resolution_width << endl;
        cout << "resolution height: " << calib.resolution_height << endl;
        cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
        cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
        cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
        cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
        cout << "radial distortion coefficients:" << endl;
        cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
        cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
        cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
        cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
        cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
        cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
        cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
        cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
        cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
        cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
        cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;

        //k4a_device_close(device);
    }
}

static void calibration_blob(uint8_t deviceIndex = 0, string filename = "calibration.json")
{
    k4a_device_t device = NULL;

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
    {
        cout << deviceIndex << ": Failed to open device" << endl;
        exit(-1);
    }

    size_t calibration_size = 0;
    k4a_buffer_result_t buffer_result = k4a_device_get_raw_calibration(device, NULL, &calibration_size);
    if (buffer_result == K4A_BUFFER_RESULT_TOO_SMALL)
    {
        vector<uint8_t> calibration_buffer = vector<uint8_t>(calibration_size);
        buffer_result = k4a_device_get_raw_calibration(device, calibration_buffer.data(), &calibration_size);
        if (buffer_result == K4A_BUFFER_RESULT_SUCCEEDED)
        {
            ofstream file(filename, ofstream::binary);
            file.write(reinterpret_cast<const char*>(&calibration_buffer[0]), (long)calibration_size);
            file.close();
            //cout << "Calibration blob for device " << (int)deviceIndex << " (serial no. " << get_serial(device)
                //<< ") is saved to " << filename << endl;
        }
        else
        {
            cout << "Failed to get calibration blob" << endl;
            exit(-1);
        }
    }
    else
    {
        cout << "Failed to get calibration blob size" << endl;
        exit(-1);
    }
}

static void print_usage()
{
    cout << "Usage: calibration_info [device_id] [output_file]" << endl;
    cout << "Using calibration_info.exe without any command line arguments will display" << endl
        << "calibration info of all connected devices in stdout. If a device_id is given" << endl
        << "(0 for default device), the calibration.json file of that device will be" << endl
        << "saved to the current directory." << endl;
}

int main(int argc, char** argv)
{
    if (argc == 1)
    {
        print_calibration();
    }
    else if (argc == 2)
    {
        calibration_blob((uint8_t)atoi(argv[1]), "calibration.json");
    }
    else if (argc == 3)
    {
        calibration_blob((uint8_t)atoi(argv[1]), argv[2]);
    }
    else
    {
        print_usage();
    }

    return 0;
}



//
// ----------------------------------------------------------------------------
// 
// カラーカメラの実行結果
// 
// 
//Found 1 connected devices :
//Opened device : 000565502412
//resolution width : 1920
//resolution height : 1080
//principal point x : 961.07861328125000000000      cx
//principal point y : 553.43408203125000000000      cy
//focal length x : 909.98956298828125000000         fx
//focal length y : 909.63812255859375000000         fy
//radial distortion coefficients :
//k1: 0.46717128157615661621
//k2 : -2.45866727828979492188
//k3 : 1.37056386470794677734
//k4 : 0.34988552331924438477
//k5 : -2.29505944252014160156
//k6 : 1.30625414848327636719
//center of distortion in Z = 1 plane, x : 0.00000000000000000000
//center of distortion in Z = 1 plane, y : 0.00000000000000000000
//tangential distortion coefficient x : 0.00136364088393747807      p1
//tangential distortion coefficient y : -0.00006751885666744784     p2
//metric radius : 0.00000000000000000000
//stop device
//close device
//
//C : \Users\student\cpp_program\kinect_calibration\kinect_calibration\x64\Release\kinect_calibration.exe(プロセス 4936) は、コード 0 で終了しました。
//デバッグが停止したときに自動的にコンソールを閉じるには、[ツール] ->[オプション] ->[デバッグ] ->[デバッグの停止時に自 動的にコンソールを閉じる] を有効にします。
//このウィンドウを閉じるには、任意のキーを押してください...
// 
// 
// ----------------------------------------------------------------------------
//

// デプスカメラの結果
//
//Found 1 connected devices :
//Opened device : 000565502412
//resolution width : 640
//resolution height : 576
//principal point x : 314.59005737304687500000
//principal point y : 332.22369384765625000000
//focal length x : 503.22808837890625000000
//focal length y : 503.35195922851562500000
//radial distortion coefficients :
//k1: 2.48467683792114257812
//k2 : 1.70563745498657226562
//k3 : 0.09044291824102401733
//k4 : 2.81460189819335937500
//k5 : 2.50367665290832519531
//k6 : 0.47529044747352600098
//center of distortion in Z = 1 plane, x : 0.00000000000000000000
//center of distortion in Z = 1 plane, y : 0.00000000000000000000
//tangential distortion coefficient x : -0.00007825181819498539
//tangential distortion coefficient y : 0.00002669491004780866
//metric radius : 0.00000000000000000000
//stop device
//close device
//
//C : \Users\student\cpp_program\kinect_calibration\kinect_calibration\x64\Release\kinect_calibration.exe(プロセス 18084)  は、コード 0 で終了しました。
//デバッグが停止したときに自動的にコンソールを閉じるには、[ツール] ->[オプション] ->[デバッグ] ->[デバッグの停止時に自 動的にコンソールを閉じる] を有効にします。
//このウィンドウを閉じるには、任意のキーを押してください...
//
