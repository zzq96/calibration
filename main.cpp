#include <iostream>  
#include <time.h>
#include <fstream>  
#include <io.h>     
#include <string>
#include <direct.h>    
#include <vector>    
//#include <axxbsolver.h>
//#include<conventionalaxxbsvdsolver.h>
using namespace std;
 
#include<opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
using namespace cv;
 
//标定板方格边长，行角点，列角点   
#define BOARD_SCALE 34
#define BOARD_HEIGHT 7
#define BOARD_WIDTH 10
//相机标定: in
//手眼标定: ex
#define RUNFUN in

void in();
void ex();
int main()
{
	RUNFUN();
	return 0;
}
 
//获取特定格式的文件名    
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
	cout << "norm" << endl;
	cout << norm(I, shouldBeIdentity) << endl;;
    return  norm(I, shouldBeIdentity) < 1e-6;    
}
void GetAllFormatFiles(string path, vector<string>& files, string format)
{
	//文件句柄      
	//long   hFile = 0;//win7使用
	intptr_t hFile = 0;//win10使用
					   //文件信息      
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*" + format).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					//files.push_back(p.assign(path).append("\\").append(fileinfo.name) );    
					GetAllFormatFiles(p.assign(path).append("\\").append(fileinfo.name), files, format);
				}
			}
			else
			{
				//files.push_back(p.assign(path).append("\\").append(fileinfo.name));//将文件路径保存
				files.push_back(p.assign(fileinfo.name));  //只保存文件名:  
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[2]),   -sin(theta[2]),
               0,       sin(theta[2]),   cos(theta[2])
               );    
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );    
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[0]),    -sin(theta[0]),      0,
               sin(theta[0]),    cos(theta[0]),       0,
               0,               0,                  1);   
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;  
	if (!isRotationMatrix(R))		//欧拉角特殊情况下会出现死锁
	{
		cout << "Euler Angle convert to RotatedMatrix failed..." << endl;
		exit(-1);
	}

    return R;
}
/**
 * 功能： 1. 通过给定的旋转矩阵计算对应的欧拉角
 * 作者： Zuo
 * 日期： 2017-10-12
**/
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(z, y, x) / CV_PI * 180;   
}
Mat R_T2HomogeneousMatrix(const Mat& R,const Mat& T)
{
	Mat HomoMtr;
	Mat_<double> R1 = (Mat_<double>(4, 3) << 
										R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
										R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
										R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
										0, 0, 0);
	Mat_<double> T1 = (Mat_<double>(4, 1) <<
										T.at<double>(0,0),
										T.at<double>(1,0),
										T.at<double>(2,0),
										1);
	cv::hconcat(R1, T1, HomoMtr);		//矩阵拼接
	return HomoMtr;
}
void HomogeneousMtr2RT(Mat& HomoMtr, Mat& R, Mat& T)
{
	//Mat R_HomoMtr = HomoMtr(Rect(0, 0, 3, 3)); //注意Rect取值
	//Mat T_HomoMtr = HomoMtr(Rect(3, 0, 1, 3));
	//R_HomoMtr.copyTo(R);
	//T_HomoMtr.copyTo(T);
	/*HomoMtr(Rect(0, 0, 3, 3)).copyTo(R);
	HomoMtr(Rect(3, 0, 1, 3)).copyTo(T);*/
	Rect R_rect(0, 0, 3, 3);
	Rect T_rect(3, 0, 1, 3);
	R = HomoMtr(R_rect);
	T = HomoMtr(T_rect);

}
//下面是标定内参的main函数
void in()
{
	//ConventionalAXXBSVDSolver a() ;
	vector<string> imageFilesName;
	vector<string> files;
	imageFilesName.clear(); files.clear();
	string filePath="Imgs_in";
	//cout << "请输入标定照片文件绝对目录路径" << endl;
	//cin >> filePath;
	string format = ".png";
	GetAllFormatFiles(filePath, imageFilesName, format);
	cout << "找到的文件有" << endl;
	for (int i = 0; i < imageFilesName.size(); i++)
	{
		files.push_back(filePath + "\\" + imageFilesName[i]);
		cout << files[i] << endl;
	}
	string calibrateDir = filePath + "\\calibrateImage";
	_mkdir(calibrateDir.c_str());
 
	//读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化   
	cout << "开始提取角点………………" << endl;
	int image_count = 0;  /* 图像数量 */
	Size image_size;  /* 图像的尺寸 */
	Size board_size = Size(BOARD_HEIGHT, BOARD_WIDTH);    /* 标定板上每行、列的角点数 */
	vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
	set<int> fail_imgs;
 
	for (int i = 0; i<files.size(); i++)
	{
		cout << files[i] << endl;
 
		Mat imageInput = imread(files[i]);
		/* 提取角点 */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			cout << "can not find chessboard corners!\n"; //找不到角点  
			continue;
		}
		else
		{
			fail_imgs.insert(i);
			//找到一幅有效的图片
			image_count++;
			if (image_count == 1)  //读入第一张图片时获取图像宽高信息  
			{
				image_size.width = imageInput.cols;
				image_size.height = imageInput.rows;
				cout << "image_size.width = " << image_size.width << endl;
				cout << "image_size.height = " << image_size.height << endl;
			}
 
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
 
			/* 亚像素精确化 */
			cornerSubPix(view_gray, image_points_buf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			//find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //对粗提取的角点进行精确化，更适合用于棋盘标定  
			/*cornerSubPix(view_gray, image_points_buf,//另一种对粗提取的角点进行精确化
							Size(5, 5),
							Size(-1, -1),
							TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
							30,        // max number of iterations
							0.1));     // min accuracy*/
			image_points_seq.push_back(image_points_buf);  //保存亚像素角点
 
														   /* 在图像上显示角点位置 */
			drawChessboardCorners(view_gray, board_size, image_points_buf, true); //用于在图片中标记角点 
			//string filePath = files[i];//写入文件
			string filePath = calibrateDir + "\\"+ imageFilesName[i] + ".jpg";
			imwrite(filePath, view_gray);
		}
	}
 
	int total = image_points_seq.size();
	cout << "共使用了" << total << "幅图片" << endl;
	cout << "角点提取完成！\n";
	cout << "开始标定………………\n";
	/*棋盘三维信息*/
	Size square_size = Size(BOARD_SCALE, BOARD_SCALE);  /* 实际测量得到的标定板上每个棋盘格的大小 */
	vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
 
	/*内外参数*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵 */
	vector<int> point_counts;  // 每幅图像中角点的数量  
	Mat distCoeffs = Mat(1, 4, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */ //由于k3是可选的所以在最后
	vector<Mat> rvecsMat;  /* 每幅图像的旋转向量 */
	vector<Mat> tvecsMat; /* 每幅图像的平移向量 */
						  /* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t<image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++)
		{
			for (j = 0; j<board_size.width; j++)
			{
				Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
 
	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	for (i = 0; i<image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
 
	/* 开始标定 */
	//CALIB_USE_INTRINSIC_GUESS;
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "标定完成！\n";

	//标定内参，外参，solve AX=XB一把梭哈
	//但是因为标定板固定在机械臂上面的时候，标定板无法覆盖全部视野，所以不能一把梭哈
	//printf("开始求解AX=XB");
	//printf("开始计算机械臂坐标系下相机坐标系的位姿\n");
	///*读取每一幅图像中机械臂的欧拉角和平移向量, 欧拉角转化为旋转矩阵*/
	//vector<Mat> R_gripper2base;
	//vector<Mat> t_gripper2base;
	//ifstream readFile(filePath + "\\data.txt");
	//for (int i = 0; i < files.size(); i++)
	//{
	//	if (fail_imgs.count(i) == 0)
	//		continue;

	//	/*calibrateHandEye是返回64F，要统一*/
	//	Mat tvecs = Mat(3, 1, CV_64FC1, Scalar::all(0));
	//	readFile >> tvecs.at<double>(0, 0);
	//	readFile >> tvecs.at<double>(1, 0);
	//	readFile >> tvecs.at<double>(2, 0);

	//	Vec3f eulerAng;
	//	Mat rmatrix;
	//	readFile >> eulerAng[0];
	//	readFile >> eulerAng[1];
	//	readFile >> eulerAng[2];

	//	//TODO:无效的图片continue
	//	
	//	cout << "第" << i << "张图像时机械手的欧拉角：" << eulerAng[0] <<" "<< eulerAng[1] <<" "<< eulerAng[2] << endl;
	//	//欧拉角转化为旋转矩阵
	//	rmatrix = eulerAnglesToRotationMatrix(eulerAng);
	//	R_gripper2base.push_back(rmatrix);
	//	cout << "第" << i << "张图像时机械手的旋转矩阵：" << rmatrix<< endl;
	//	cout << "第" << i << "张图像时机械手的平移向量为：" << tvecs.at<double>(0, 0) << " " << tvecs.at<double>(1, 0) << " " << tvecs.at<double>(2, 0) << endl << endl;
	//	t_gripper2base.push_back(tvecs);
	//}
	//readFile.close();
	///**/
	//vector<Mat> rotation(rvecsMat.size()); /* 保存每幅图像的旋转矩阵 */
	//Mat R_target2gripper;
	//Mat t_target2gripper;
	///* 将旋转向量转换为相对应的旋转矩阵 */
	//for(int i = 0; i < rvecsMat.size();i++)
	//{
	//	Rodrigues(rvecsMat[i], rotation[i]);
	//	/*由于calibrateHandEye是求eye in hand的，我们这里是求eye to hand所以要求一个逆。同时平移向量要取个负号 */
	//	/*其实我也不知道calibratecamera求出来的标定板和相机之间的旋转关系是谁到谁的，反正求不求逆都试了一下，好像不求逆的时候比较正确*/
	//	//invert(rotation[i], rotation[i], DECOMP_LU);
	//	//tvecsMat[i] = tvecsMat[i] * -1;
	//}
	////解AX=XB
	//calibrateHandEye(R_gripper2base, t_gripper2base,
	//	rotation,
	//	tvecsMat,
	//	R_target2gripper,
	//	t_target2gripper,
	//	CALIB_HAND_EYE_TSAI
	//);
	///*标定板到机械手的旋转矩阵和平移向量*/
	//cout << "标定板到机械手的旋转矩阵和平移向量" << endl;
	//cout << R_target2gripper << endl;
	//cout << t_target2gripper << endl;
	///*求出camera到base的旋转矩阵和平移向量*/
	//Mat R_cam2base0 = R_gripper2base[0] * R_target2gripper * rotation[0];
	//Mat R_cam2base1 = R_gripper2base[1] * R_target2gripper * rotation[1];
	//Mat t_cam2base0 = t_gripper2base[0] + t_target2gripper + tvecsMat[0];
	//Mat t_cam2base1 = t_gripper2base[1] + t_target2gripper + tvecsMat[1];
	//cout << "finally R, t0" << endl;
	//cout << R_cam2base0 << endl;
	//cout << R_cam2base1 << endl;
	//cout << "finally R, t1" << endl;
	//cout << t_cam2base0 << endl;
	//cout << t_cam2base1 << endl;

	//对标定结果进行评价
	string txtResult = filePath + "\\calibrateImage\\caliberation_result.txt";
	ofstream fout(txtResult);  /* 保存标定结果的文件 */
 
	double total_err = 0.0; /* 所有图像的平均误差的总和 */
	double err = 0.0; /* 每幅图像的平均误差 */
	vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
	cout << "\t每幅图像的标定误差：\n";
	fout << "每幅图像的标定误差：\n";
	for (i = 0; i<image_count; i++)
	{
		vector<Point3f> tempPointSet = object_points[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
 
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
 
	//保存定标结果      
	cout << "开始保存定标结果………………" << endl;
	cv::Mat rotation_matrix;
	fout << "相机内参数矩阵：" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "畸变系数：\n";
	fout << distCoeffs << endl << endl << endl;
	cout << distCoeffs.cols << endl;
	for (int i = 0; i<image_count; i++)
	{
		Rodrigues(rvecsMat[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << rvecsMat[i] << endl;
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix<< endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << tvecsMat[i] << endl << endl;
	}
	fout << endl;
 
	//保存相机内参数矩阵和畸变系数和相机距离到xml
	cout << "开始保存相机内参数矩阵和畸变系数………………" << endl;
	string xmlResult = filePath + "\\calibrateImage\\caliberation_camera.xml";
	FileStorage fs(xmlResult, FileStorage::WRITE); //创建XML文件  
	fs << "zConst" << 100.0;
	fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
	fs.release();
	//保存平移矩阵和旋转矩阵和s到xml
	string xml2Result = filePath + "\\calibrateImage\\solvePnP_camera.xml";
	FileStorage fs2(xml2Result, FileStorage::WRITE); //创建XML文件 
	double s = 100.0;
	fs2 << "s" << s;
	fs2 << "rotation_matrix" << rotation_matrix << "tvecsMat" << tvecsMat[0];
	fs2.release();
	cout << "保存完成" << endl;
 
	//保存矫正图像
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	cout << "保存矫正图像" << endl;
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);

	cout << mapx.at<float>(0, 0)<< endl;
	cout << mapy.at<float>(0, 0) << endl;
	for (int i = 0; i != image_count; i++)
	{
		cout << "Frame #" << i + 1 << "..." << endl;
		Mat imageSource = imread(files[i]);
		Mat newimage = imageSource.clone();
		//另一种不需要转换矩阵的方式  
		undistort(imageSource,newimage,cameraMatrix,distCoeffs);  
		//remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);//效率更高
		string imageFilePath = calibrateDir + "\\" + imageFilesName[i] +"_d"+ ".png";
		imwrite(imageFilePath, newimage);
	}
	cout << "保存结束" << endl;
	string depth_dir = "D:\\Code\\c++\\Azure Kinect\\imgs\\img0_depthcolor.png";
	Mat depth = imread(depth_dir);
	cout << depth.rows << " " << depth.cols << endl;
	Mat depth_new = depth.clone();
	//undistort(depth,depth_new,cameraMatrix,distCoeffs);  
	//imwrite("test.png", depth_new);
 
	cout << "全部工作结束" << endl;
	int key = waitKey(0);
	return;
}
//下面标定外参
void ex()
{
	//ConventionalAXXBSVDSolver a() ;
	vector<string> imageFilesName;
	vector<string> files;
	imageFilesName.clear(); files.clear();
	string filePath="Imgs_ex";
	//cout << "请输入标定照片文件绝对目录路径" << endl;
	//cin >> filePath;
	string format = ".png";
	GetAllFormatFiles(filePath, imageFilesName, format);
	cout << "找到的文件有" << endl;
	for (int i = 0; i < imageFilesName.size(); i++)
	{
		files.push_back(filePath + "\\" + imageFilesName[i]);
		cout << files[i] << endl;
	}
	string calibrateDir = filePath + "\\calibrateImage";
	_mkdir(calibrateDir.c_str());
 
	//读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化   
	cout << "开始提取角点………………" << endl;
	int image_count = 0;  /* 图像数量 */
	Size image_size;  /* 图像的尺寸 */
	Size board_size = Size(BOARD_HEIGHT, BOARD_WIDTH);    /* 标定板上每行、列的角点数 */
	vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
	set<int> fail_imgs;
 
	for (int i = 0; i<files.size(); i++)
	{
		cout << files[i] << endl;
 
		Mat imageInput = imread(files[i]);
		/* 提取角点 */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			fail_imgs.insert(i);
			cout << "can not find chessboard corners!\n"; //找不到角点  
			continue;
		}
		else
		{
			//找到一幅有效的图片
			image_count++;
			if (image_count == 1)  //读入第一张图片时获取图像宽高信息  
			{
				image_size.width = imageInput.cols;
				image_size.height = imageInput.rows;
				cout << "image_size.width = " << image_size.width << endl;
				cout << "image_size.height = " << image_size.height << endl;
			}
 
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
 
			/* 亚像素精确化 */
			cornerSubPix(view_gray, image_points_buf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			//find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //对粗提取的角点进行精确化，更适合用于棋盘标定  
			/*cornerSubPix(view_gray, image_points_buf,//另一种对粗提取的角点进行精确化
							Size(5, 5),
							Size(-1, -1),
							TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
							30,        // max number of iterations
							0.1));     // min accuracy*/
			image_points_seq.push_back(image_points_buf);  //保存亚像素角点
 
														   /* 在图像上显示角点位置 */
			drawChessboardCorners(view_gray, board_size, image_points_buf, true); //用于在图片中标记角点 
			//string filePath = files[i];//写入文件
			string filePath = calibrateDir + "\\"+ imageFilesName[i] + ".jpg";
			imwrite(filePath, view_gray);
		}
	}
 
	int total = image_points_seq.size();
	cout << "共使用了" << total << "幅图片" << endl;
	cout << "角点提取完成！\n";
	cout << "开始标定………………\n";
	/*棋盘三维信息*/
	Size square_size = Size(BOARD_SCALE, BOARD_SCALE);  /* 实际测量得到的标定板上每个棋盘格的大小 */
	vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
 
	/*内外参数*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵 */
	vector<int> point_counts;  // 每幅图像中角点的数量  
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */ //由于k3是可选的所以在最后
	vector<Mat> R_cam2target(image_count);  /* 每幅图像的旋转向量 */
	vector<Mat> t_cam2target(image_count); /* 每幅图像的平移向量 */
						  /* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t<image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++)
		{
			for (j = 0; j<board_size.width; j++)
			{
				Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	cout << "image0中标定板第一个点和最后一个点的坐标" << endl;
	cout << object_points[0][0] << endl << object_points[0][1] << endl;
	cout << image_points_seq[0][0] << endl << image_points_seq[0][1] << endl;
 
	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	for (i = 0; i<image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
 
	///* 开始标定 */
	////CALIB_USE_INTRINSIC_GUESS;
	//calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	//cout << "标定完成！\n";
	//读取内参和畸变系数
	cout << "开始读取相机内参数矩阵和畸变系数………………" << endl;
	string xmlResult="Imgs_in\\calibrateImage\\caliberation_camera.xml";
	FileStorage fs(xmlResult, FileStorage::READ); //读取XML文件  
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"]>> distCoeffs;
	fs.release();
	cout << "内参矩阵" << endl;
	cout << cameraMatrix << endl;
	cout << "畸变系数" << endl;
	cout << distCoeffs << endl;

	//计算外参
	printf("开始计算外参");
	for (int i = 0; i < object_points.size(); i++)
	{
		solvePnP(object_points[i], image_points_seq[i], cameraMatrix, distCoeffs, R_cam2target[i], t_cam2target[i]);
		//cout << i << endl;
		////if (R_cam2target[i].at<double>(0, 0) > 0)
		////	R_cam2target[i].at<double>(0, 0) *= -1, R_cam2target[i].at<double>(1, 0) *= -1;
		//cout << R_cam2target[i] << endl;
		//cout << t_cam2target[i] << endl;
	}
	printf("计算外参完成");

	//求解AX=XB
	printf("开始求解AX=XB");
	printf("开始计算机械臂坐标系下相机坐标系的位姿\n");
	/*读取每一幅图像中机械臂的欧拉角和平移向量, 欧拉角转化为旋转矩阵*/
	vector<Mat> R_gripper2base;
	vector<Mat> t_gripper2base;
	ifstream readFile(filePath + "\\Data.txt");
	for (int i = 0; i < files.size(); i++)
	{
		/*calibrateHandEye是返回64F，要统一*/
		Mat tvecs = Mat(3, 1, CV_64FC1, Scalar::all(0));
		readFile >> tvecs.at<double>(0, 0);
		readFile >> tvecs.at<double>(1, 0);
		readFile >> tvecs.at<double>(2, 0);

		Vec3f eulerAng;
		Mat rmatrix;
		readFile >> eulerAng[0] ;
		readFile >> eulerAng[1];
		readFile >> eulerAng[2];
		cout << files[i] << endl;
		cout << "机械手的欧拉角：" << eulerAng[0] <<" "<< eulerAng[1] <<" "<< eulerAng[2] << endl;
		eulerAng = eulerAng * CV_PI / 180.0;

		if (fail_imgs.count(i) != 0)
			continue;

		//欧拉角转化为旋转矩阵
		rmatrix = eulerAnglesToRotationMatrix(eulerAng);
		R_gripper2base.push_back(rmatrix);
		cout << "机械手的旋转矩阵：\n" << rmatrix<<" "<<isRotationMatrix(rmatrix)<<endl;
		cout << "机械手的平移向量为：" << tvecs.at<double>(0, 0) << " " << tvecs.at<double>(1, 0) << " " << tvecs.at<double>(2, 0) << endl << endl;
		Mat rotationnew;
		Vec3f  e = rotationMatrixToEulerAngles(rmatrix);
		cout << "新的欧拉角" << e << endl;
		t_gripper2base.push_back(tvecs);
	}
	readFile.close();
	/**/
	Mat R_target2gripper;
	Mat t_target2gripper;
	/* 将旋转向量转换为相对应的旋转矩阵 */
	for(int i = 0; i < R_cam2target.size();i++)
	{
		Rodrigues(R_cam2target[i], R_cam2target[i]);
		cout << "第" << i << "个标定板到相机的旋转矩阵" << R_cam2target[i] << endl;
		Mat Homo = R_T2HomogeneousMatrix(R_cam2target[i], t_cam2target[i]);
		invert(Homo, Homo);
		HomogeneousMtr2RT(Homo, R_cam2target[i], t_cam2target[i]);
		cout << "第" << i << "个相机到标定板的旋转矩阵" << R_cam2target[i] << endl;
		/*由于calibrateHandEye是求eye in hand的，我们这里是求eye to hand所以要求一个逆。同时平移向量要取个负号 */
		/*其实我也不知道calibratecamera求出来的标定板和相机之间的旋转关系是谁到谁的，反正求不求逆都试了一下，好像不求逆的时候比较正确*/
		//invert(R_cam2target[i], R_cam2target[i], DECOMP_LU);
		//t_cam2target[i] = t_cam2target[i] * -1;

	//	Vec3f e = rotationMatrixToEulerAngles(R_cam2target[i]);
	//	cout << t_cam2target[i].at<double>(0, 0) << "," << t_cam2target[i].at<double>(1, 0) << "," << t_cam2target[i].at<double>(2, 0) << ","
	//		<< e[0] << "," << e[1] << "," << e[2] <<","<< endl;

	}
	//解AX=XB
	calibrateHandEye(R_gripper2base, t_gripper2base,
		R_cam2target,
		t_cam2target,
		R_target2gripper,
		t_target2gripper,
		CALIB_HAND_EYE_HORAUD
		//CALIB_HAND_EYE_ANDREFF
		//CALIB_HAND_EYE_PARK
		//CALIB_HAND_EYE_TSAI
	);
	//R_target2gripper.at<double>(0, 0) = -1;
	//R_target2gripper.at<double>(0, 1) = 0;
	//R_target2gripper.at<double>(0, 2) = 0;
	//R_target2gripper.at<double>(1, 0) = 0;
	//R_target2gripper.at<double>(1, 1) = -1;
	//R_target2gripper.at<double>(1, 2) = 0;
	//R_target2gripper.at<double>(2, 0) = 0;
	//R_target2gripper.at<double>(2, 1) = 0;
	//R_target2gripper.at<double>(2, 2) = 1;
	/*求出单应矩阵*/
	vector<Mat> Homo_gripper2base, Homo_cam2target;
	Mat  Homo_target2gripper;
	Homo_target2gripper = R_T2HomogeneousMatrix(R_target2gripper, t_target2gripper);
	for (int i = 0; i < image_count; i++)
	{
		Homo_gripper2base.push_back(R_T2HomogeneousMatrix(R_gripper2base[i], t_gripper2base[i]));
		Homo_cam2target.push_back(R_T2HomogeneousMatrix(R_cam2target[i], t_cam2target[i]));

		cout << "图像"<<i<<"的旋转矩阵和平移向量" << endl;
		cout << R_cam2target[i] << t_cam2target[i] << endl;
	}
	/*测试*/
	/*标定板到机械手的旋转矩阵和平移向量*/
	cout << "标定板到机械手的单应矩阵" << endl;
	cout << Homo_target2gripper << endl;
	Mat Homo_cam2basesum = Mat(4, 4, CV_64FC1, Scalar::all(0)); ;
	for (int i = 0; i < image_count; i++)
	{
		Mat Homo_cam2base = Homo_gripper2base[i] * Homo_target2gripper * Homo_cam2target[i];
		Homo_cam2basesum += Homo_cam2base;

		cout << files[i] << endl;
		Mat R, T;
		HomogeneousMtr2RT(Homo_cam2base, R, T);
		cout << "is_rotation" << " " << isRotationMatrix(R) << endl;
		cout << Homo_cam2base << endl;

		//cout << "第" << i << "个相机到标定板的单应矩阵" << endl;
		//cout << Homo_cam2target[i] << endl;
		//cout << "第" << i << "个抓手到基座的单应矩阵" << endl;
		//cout << Homo_gripper2base[i] << endl;
		cout << endl;

	}
	cout << "finally cam2base" << endl;
	Homo_cam2basesum /= image_count;
	cout << Homo_cam2basesum<< endl;
	Mat R, T;
	HomogeneousMtr2RT(Homo_cam2basesum, R, T);
	cout << "is_rotation" << " " << isRotationMatrix(R) << endl;
	cout << "R:" << R << endl;
	Mat w, u, vt;
	cv::SVDecomp(R, w, u, vt);
    Mat I = Mat::eye(3,3, R.type());
	Mat new_R = u * I * vt;
	cout << "is_rotation " << isRotationMatrix(new_R) << endl;
	cout << new_R << endl;
	Homo_cam2basesum = R_T2HomogeneousMatrix(new_R, T);

	string xmlResult2 = filePath + "\\calibrateImage\\Homo_cam2base.xml";
	FileStorage fs2(xmlResult2, FileStorage::WRITE); //创建XML文件  
	fs2 << "Homo_cam2base" << Homo_cam2basesum;
	fs2.release();
 
	////对标定结果进行评价
	//string txtResult = filePath + "\\calibrateImage\\caliberation_result.txt";
	//ofstream fout(txtResult);  /* 保存标定结果的文件 */
 
	//double total_err = 0.0; /* 所有图像的平均误差的总和 */
	//double err = 0.0; /* 每幅图像的平均误差 */
	//vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
	//cout << "\t每幅图像的标定误差：\n";
	//fout << "每幅图像的标定误差：\n";
	//for (i = 0; i<image_count; i++)
	//{
	//	vector<Point3f> tempPointSet = object_points[i];
	//	/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
	//	projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
	//	/* 计算新的投影点和旧的投影点之间的误差*/
	//	vector<Point2f> tempImagePoint = image_points_seq[i];
	//	Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
	//	Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
	//	for (int j = 0; j < tempImagePoint.size(); j++)
	//	{
	//		image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
	//		tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
	//	}
	//	err = norm(image_points2Mat, tempImagePointMat, NORM_L2);

	//	total_err += err /= point_counts[i];
	//	cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	//	fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	//}
 
	//cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	//fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
 
	////保存定标结果      
	//cout << "开始保存定标结果………………" << endl;
	//cv::Mat rotation_matrix;
	//fout << "相机内参数矩阵：" << endl;
	//fout << cameraMatrix << endl << endl;
	//fout << "畸变系数：\n";
	//fout << distCoeffs << endl << endl << endl;
	//cout << distCoeffs.cols << endl;
	//for (int i = 0; i<image_count; i++)
	//{
	//	Rodrigues(rvecsMat[i], rotation_matrix);
	//	fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
	//	fout << rvecsMat[i] << endl;
	//	fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
	//	fout << rotation_matrix<< endl;
	//	fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
	//	fout << tvecsMat[i] << endl << endl;
	//}
	//fout << endl;
 
	////保存相机内参数矩阵和畸变系数和相机距离到xml
	//cout << "开始保存相机内参数矩阵和畸变系数………………" << endl;
	//string xmlResult = filePath + "\\calibrateImage\\caliberation_camera.xml";
	//FileStorage fs(xmlResult, FileStorage::WRITE); //创建XML文件  
	//fs << "zConst" << 100.0;
	//fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
	//fs.release();
	////保存平移矩阵和旋转矩阵和s到xml
	//string xml2Result = filePath + "\\calibrateImage\\solvePnP_camera.xml";
	//FileStorage fs2(xml2Result, FileStorage::WRITE); //创建XML文件 
	//double s = 100.0;
	//fs2 << "s" << s;
	//fs2 << "rotation_matrix" << rotation_matrix << "tvecsMat" << tvecsMat[0];
	//fs2.release();
	//cout << "保存完成" << endl;
 
	////保存矫正图像
	//Mat mapx = Mat(image_size, CV_32FC1);
	//Mat mapy = Mat(image_size, CV_32FC1);
	//Mat R = Mat::eye(3, 3, CV_32F);
	//cout << "保存矫正图像" << endl;
	//initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);

	//cout << mapx.at<float>(0, 0)<< endl;
	//cout << mapy.at<float>(0, 0) << endl;
	//for (int i = 0; i != image_count; i++)
	//{
	//	cout << "Frame #" << i + 1 << "..." << endl;
	//	Mat imageSource = imread(files[i]);
	//	Mat newimage = imageSource.clone();
	//	//另一种不需要转换矩阵的方式  
	//	undistort(imageSource,newimage,cameraMatrix,distCoeffs);  
	//	//remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);//效率更高
	//	string imageFilePath = calibrateDir + "\\" + imageFilesName[i] +"_d"+ ".png";
	//	imwrite(imageFilePath, newimage);
	//}
	//cout << "保存结束" << endl;
	//string depth_dir = "D:\\Code\\c++\\Azure Kinect\\imgs\\img0_depthcolor.png";
	//Mat depth = imread(depth_dir);
	//cout << depth.rows << " " << depth.cols << endl;
	//Mat depth_new = depth.clone();
	//undistort(depth,depth_new,cameraMatrix,distCoeffs);  
	//imwrite("test.png", depth_new);

 
	//cout << "全部工作结束" << endl;
	int key = waitKey(0);
	return;
}