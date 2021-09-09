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
 
//�궨�巽��߳����нǵ㣬�нǵ�   
#define BOARD_SCALE 34
#define BOARD_HEIGHT 7
#define BOARD_WIDTH 10
//����궨: in
//���۱궨: ex
#define RUNFUN in

void in();
void ex();
int main()
{
	RUNFUN();
	return 0;
}
 
//��ȡ�ض���ʽ���ļ���    
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
	//�ļ����      
	//long   hFile = 0;//win7ʹ��
	intptr_t hFile = 0;//win10ʹ��
					   //�ļ���Ϣ      
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
				//files.push_back(p.assign(path).append("\\").append(fileinfo.name));//���ļ�·������
				files.push_back(p.assign(fileinfo.name));  //ֻ�����ļ���:  
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
	if (!isRotationMatrix(R))		//ŷ������������»��������
	{
		cout << "Euler Angle convert to RotatedMatrix failed..." << endl;
		exit(-1);
	}

    return R;
}
/**
 * ���ܣ� 1. ͨ����������ת��������Ӧ��ŷ����
 * ���ߣ� Zuo
 * ���ڣ� 2017-10-12
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
	cv::hconcat(R1, T1, HomoMtr);		//����ƴ��
	return HomoMtr;
}
void HomogeneousMtr2RT(Mat& HomoMtr, Mat& R, Mat& T)
{
	//Mat R_HomoMtr = HomoMtr(Rect(0, 0, 3, 3)); //ע��Rectȡֵ
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
//�����Ǳ궨�ڲε�main����
void in()
{
	//ConventionalAXXBSVDSolver a() ;
	vector<string> imageFilesName;
	vector<string> files;
	imageFilesName.clear(); files.clear();
	string filePath="Imgs_in";
	//cout << "������궨��Ƭ�ļ�����Ŀ¼·��" << endl;
	//cin >> filePath;
	string format = ".png";
	GetAllFormatFiles(filePath, imageFilesName, format);
	cout << "�ҵ����ļ���" << endl;
	for (int i = 0; i < imageFilesName.size(); i++)
	{
		files.push_back(filePath + "\\" + imageFilesName[i]);
		cout << files[i] << endl;
	}
	string calibrateDir = filePath + "\\calibrateImage";
	_mkdir(calibrateDir.c_str());
 
	//��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��   
	cout << "��ʼ��ȡ�ǵ㡭����������" << endl;
	int image_count = 0;  /* ͼ������ */
	Size image_size;  /* ͼ��ĳߴ� */
	Size board_size = Size(BOARD_HEIGHT, BOARD_WIDTH);    /* �궨����ÿ�С��еĽǵ��� */
	vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */
	set<int> fail_imgs;
 
	for (int i = 0; i<files.size(); i++)
	{
		cout << files[i] << endl;
 
		Mat imageInput = imread(files[i]);
		/* ��ȡ�ǵ� */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�  
			continue;
		}
		else
		{
			fail_imgs.insert(i);
			//�ҵ�һ����Ч��ͼƬ
			image_count++;
			if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ������Ϣ  
			{
				image_size.width = imageInput.cols;
				image_size.height = imageInput.rows;
				cout << "image_size.width = " << image_size.width << endl;
				cout << "image_size.height = " << image_size.height << endl;
			}
 
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
 
			/* �����ؾ�ȷ�� */
			cornerSubPix(view_gray, image_points_buf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			//find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ�������ʺ��������̱궨  
			/*cornerSubPix(view_gray, image_points_buf,//��һ�ֶԴ���ȡ�Ľǵ���о�ȷ��
							Size(5, 5),
							Size(-1, -1),
							TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
							30,        // max number of iterations
							0.1));     // min accuracy*/
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
 
														   /* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, true); //������ͼƬ�б�ǽǵ� 
			//string filePath = files[i];//д���ļ�
			string filePath = calibrateDir + "\\"+ imageFilesName[i] + ".jpg";
			imwrite(filePath, view_gray);
		}
	}
 
	int total = image_points_seq.size();
	cout << "��ʹ����" << total << "��ͼƬ" << endl;
	cout << "�ǵ���ȡ��ɣ�\n";
	cout << "��ʼ�궨������������\n";
	/*������ά��Ϣ*/
	Size square_size = Size(BOARD_SCALE, BOARD_SCALE);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	vector<vector<Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
 
	/*�������*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
	vector<int> point_counts;  // ÿ��ͼ���нǵ������  
	Mat distCoeffs = Mat(1, 4, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */ //����k3�ǿ�ѡ�����������
	vector<Mat> rvecsMat;  /* ÿ��ͼ�����ת���� */
	vector<Mat> tvecsMat; /* ÿ��ͼ���ƽ������ */
						  /* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t<image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++)
		{
			for (j = 0; j<board_size.width; j++)
			{
				Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
 
	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
	for (i = 0; i<image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
 
	/* ��ʼ�궨 */
	//CALIB_USE_INTRINSIC_GUESS;
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "�궨��ɣ�\n";

	//�궨�ڲΣ���Σ�solve AX=XBһ�����
	//������Ϊ�궨��̶��ڻ�е�������ʱ�򣬱궨���޷�����ȫ����Ұ�����Բ���һ�����
	//printf("��ʼ���AX=XB");
	//printf("��ʼ�����е������ϵ���������ϵ��λ��\n");
	///*��ȡÿһ��ͼ���л�е�۵�ŷ���Ǻ�ƽ������, ŷ����ת��Ϊ��ת����*/
	//vector<Mat> R_gripper2base;
	//vector<Mat> t_gripper2base;
	//ifstream readFile(filePath + "\\data.txt");
	//for (int i = 0; i < files.size(); i++)
	//{
	//	if (fail_imgs.count(i) == 0)
	//		continue;

	//	/*calibrateHandEye�Ƿ���64F��Ҫͳһ*/
	//	Mat tvecs = Mat(3, 1, CV_64FC1, Scalar::all(0));
	//	readFile >> tvecs.at<double>(0, 0);
	//	readFile >> tvecs.at<double>(1, 0);
	//	readFile >> tvecs.at<double>(2, 0);

	//	Vec3f eulerAng;
	//	Mat rmatrix;
	//	readFile >> eulerAng[0];
	//	readFile >> eulerAng[1];
	//	readFile >> eulerAng[2];

	//	//TODO:��Ч��ͼƬcontinue
	//	
	//	cout << "��" << i << "��ͼ��ʱ��е�ֵ�ŷ���ǣ�" << eulerAng[0] <<" "<< eulerAng[1] <<" "<< eulerAng[2] << endl;
	//	//ŷ����ת��Ϊ��ת����
	//	rmatrix = eulerAnglesToRotationMatrix(eulerAng);
	//	R_gripper2base.push_back(rmatrix);
	//	cout << "��" << i << "��ͼ��ʱ��е�ֵ���ת����" << rmatrix<< endl;
	//	cout << "��" << i << "��ͼ��ʱ��е�ֵ�ƽ������Ϊ��" << tvecs.at<double>(0, 0) << " " << tvecs.at<double>(1, 0) << " " << tvecs.at<double>(2, 0) << endl << endl;
	//	t_gripper2base.push_back(tvecs);
	//}
	//readFile.close();
	///**/
	//vector<Mat> rotation(rvecsMat.size()); /* ����ÿ��ͼ�����ת���� */
	//Mat R_target2gripper;
	//Mat t_target2gripper;
	///* ����ת����ת��Ϊ���Ӧ����ת���� */
	//for(int i = 0; i < rvecsMat.size();i++)
	//{
	//	Rodrigues(rvecsMat[i], rotation[i]);
	//	/*����calibrateHandEye����eye in hand�ģ�������������eye to hand����Ҫ��һ���档ͬʱƽ������Ҫȡ������ */
	//	/*��ʵ��Ҳ��֪��calibratecamera������ı궨������֮�����ת��ϵ��˭��˭�ģ����������涼����һ�£����������ʱ��Ƚ���ȷ*/
	//	//invert(rotation[i], rotation[i], DECOMP_LU);
	//	//tvecsMat[i] = tvecsMat[i] * -1;
	//}
	////��AX=XB
	//calibrateHandEye(R_gripper2base, t_gripper2base,
	//	rotation,
	//	tvecsMat,
	//	R_target2gripper,
	//	t_target2gripper,
	//	CALIB_HAND_EYE_TSAI
	//);
	///*�궨�嵽��е�ֵ���ת�����ƽ������*/
	//cout << "�궨�嵽��е�ֵ���ת�����ƽ������" << endl;
	//cout << R_target2gripper << endl;
	//cout << t_target2gripper << endl;
	///*���camera��base����ת�����ƽ������*/
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

	//�Ա궨�����������
	string txtResult = filePath + "\\calibrateImage\\caliberation_result.txt";
	ofstream fout(txtResult);  /* ����궨������ļ� */
 
	double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0; /* ÿ��ͼ���ƽ����� */
	vector<Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	cout << "\tÿ��ͼ��ı궨��\n";
	fout << "ÿ��ͼ��ı궨��\n";
	for (i = 0; i<image_count; i++)
	{
		vector<Point3f> tempPointSet = object_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
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
		cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
 
	cout << "����ƽ����" << total_err / image_count << "����" << endl;
	fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
 
	//���涨����      
	cout << "��ʼ���涨����������������" << endl;
	cv::Mat rotation_matrix;
	fout << "����ڲ�������" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "����ϵ����\n";
	fout << distCoeffs << endl << endl << endl;
	cout << distCoeffs.cols << endl;
	for (int i = 0; i<image_count; i++)
	{
		Rodrigues(rvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		fout << rvecsMat[i] << endl;
		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		fout << rotation_matrix<< endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		fout << tvecsMat[i] << endl << endl;
	}
	fout << endl;
 
	//��������ڲ�������ͻ���ϵ����������뵽xml
	cout << "��ʼ��������ڲ�������ͻ���ϵ��������������" << endl;
	string xmlResult = filePath + "\\calibrateImage\\caliberation_camera.xml";
	FileStorage fs(xmlResult, FileStorage::WRITE); //����XML�ļ�  
	fs << "zConst" << 100.0;
	fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
	fs.release();
	//����ƽ�ƾ������ת�����s��xml
	string xml2Result = filePath + "\\calibrateImage\\solvePnP_camera.xml";
	FileStorage fs2(xml2Result, FileStorage::WRITE); //����XML�ļ� 
	double s = 100.0;
	fs2 << "s" << s;
	fs2 << "rotation_matrix" << rotation_matrix << "tvecsMat" << tvecsMat[0];
	fs2.release();
	cout << "�������" << endl;
 
	//�������ͼ��
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	cout << "�������ͼ��" << endl;
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);

	cout << mapx.at<float>(0, 0)<< endl;
	cout << mapy.at<float>(0, 0) << endl;
	for (int i = 0; i != image_count; i++)
	{
		cout << "Frame #" << i + 1 << "..." << endl;
		Mat imageSource = imread(files[i]);
		Mat newimage = imageSource.clone();
		//��һ�ֲ���Ҫת������ķ�ʽ  
		undistort(imageSource,newimage,cameraMatrix,distCoeffs);  
		//remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);//Ч�ʸ���
		string imageFilePath = calibrateDir + "\\" + imageFilesName[i] +"_d"+ ".png";
		imwrite(imageFilePath, newimage);
	}
	cout << "�������" << endl;
	string depth_dir = "D:\\Code\\c++\\Azure Kinect\\imgs\\img0_depthcolor.png";
	Mat depth = imread(depth_dir);
	cout << depth.rows << " " << depth.cols << endl;
	Mat depth_new = depth.clone();
	//undistort(depth,depth_new,cameraMatrix,distCoeffs);  
	//imwrite("test.png", depth_new);
 
	cout << "ȫ����������" << endl;
	int key = waitKey(0);
	return;
}
//����궨���
void ex()
{
	//ConventionalAXXBSVDSolver a() ;
	vector<string> imageFilesName;
	vector<string> files;
	imageFilesName.clear(); files.clear();
	string filePath="Imgs_ex";
	//cout << "������궨��Ƭ�ļ�����Ŀ¼·��" << endl;
	//cin >> filePath;
	string format = ".png";
	GetAllFormatFiles(filePath, imageFilesName, format);
	cout << "�ҵ����ļ���" << endl;
	for (int i = 0; i < imageFilesName.size(); i++)
	{
		files.push_back(filePath + "\\" + imageFilesName[i]);
		cout << files[i] << endl;
	}
	string calibrateDir = filePath + "\\calibrateImage";
	_mkdir(calibrateDir.c_str());
 
	//��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��   
	cout << "��ʼ��ȡ�ǵ㡭����������" << endl;
	int image_count = 0;  /* ͼ������ */
	Size image_size;  /* ͼ��ĳߴ� */
	Size board_size = Size(BOARD_HEIGHT, BOARD_WIDTH);    /* �궨����ÿ�С��еĽǵ��� */
	vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */
	set<int> fail_imgs;
 
	for (int i = 0; i<files.size(); i++)
	{
		cout << files[i] << endl;
 
		Mat imageInput = imread(files[i]);
		/* ��ȡ�ǵ� */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			fail_imgs.insert(i);
			cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�  
			continue;
		}
		else
		{
			//�ҵ�һ����Ч��ͼƬ
			image_count++;
			if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ������Ϣ  
			{
				image_size.width = imageInput.cols;
				image_size.height = imageInput.rows;
				cout << "image_size.width = " << image_size.width << endl;
				cout << "image_size.height = " << image_size.height << endl;
			}
 
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
 
			/* �����ؾ�ȷ�� */
			cornerSubPix(view_gray, image_points_buf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			//find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ�������ʺ��������̱궨  
			/*cornerSubPix(view_gray, image_points_buf,//��һ�ֶԴ���ȡ�Ľǵ���о�ȷ��
							Size(5, 5),
							Size(-1, -1),
							TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
							30,        // max number of iterations
							0.1));     // min accuracy*/
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
 
														   /* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, true); //������ͼƬ�б�ǽǵ� 
			//string filePath = files[i];//д���ļ�
			string filePath = calibrateDir + "\\"+ imageFilesName[i] + ".jpg";
			imwrite(filePath, view_gray);
		}
	}
 
	int total = image_points_seq.size();
	cout << "��ʹ����" << total << "��ͼƬ" << endl;
	cout << "�ǵ���ȡ��ɣ�\n";
	cout << "��ʼ�궨������������\n";
	/*������ά��Ϣ*/
	Size square_size = Size(BOARD_SCALE, BOARD_SCALE);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	vector<vector<Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
 
	/*�������*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
	vector<int> point_counts;  // ÿ��ͼ���нǵ������  
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */ //����k3�ǿ�ѡ�����������
	vector<Mat> R_cam2target(image_count);  /* ÿ��ͼ�����ת���� */
	vector<Mat> t_cam2target(image_count); /* ÿ��ͼ���ƽ������ */
						  /* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t<image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++)
		{
			for (j = 0; j<board_size.width; j++)
			{
				Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	cout << "image0�б궨���һ��������һ���������" << endl;
	cout << object_points[0][0] << endl << object_points[0][1] << endl;
	cout << image_points_seq[0][0] << endl << image_points_seq[0][1] << endl;
 
	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
	for (i = 0; i<image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
 
	///* ��ʼ�궨 */
	////CALIB_USE_INTRINSIC_GUESS;
	//calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	//cout << "�궨��ɣ�\n";
	//��ȡ�ڲκͻ���ϵ��
	cout << "��ʼ��ȡ����ڲ�������ͻ���ϵ��������������" << endl;
	string xmlResult="Imgs_in\\calibrateImage\\caliberation_camera.xml";
	FileStorage fs(xmlResult, FileStorage::READ); //��ȡXML�ļ�  
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"]>> distCoeffs;
	fs.release();
	cout << "�ڲξ���" << endl;
	cout << cameraMatrix << endl;
	cout << "����ϵ��" << endl;
	cout << distCoeffs << endl;

	//�������
	printf("��ʼ�������");
	for (int i = 0; i < object_points.size(); i++)
	{
		solvePnP(object_points[i], image_points_seq[i], cameraMatrix, distCoeffs, R_cam2target[i], t_cam2target[i]);
		//cout << i << endl;
		////if (R_cam2target[i].at<double>(0, 0) > 0)
		////	R_cam2target[i].at<double>(0, 0) *= -1, R_cam2target[i].at<double>(1, 0) *= -1;
		//cout << R_cam2target[i] << endl;
		//cout << t_cam2target[i] << endl;
	}
	printf("����������");

	//���AX=XB
	printf("��ʼ���AX=XB");
	printf("��ʼ�����е������ϵ���������ϵ��λ��\n");
	/*��ȡÿһ��ͼ���л�е�۵�ŷ���Ǻ�ƽ������, ŷ����ת��Ϊ��ת����*/
	vector<Mat> R_gripper2base;
	vector<Mat> t_gripper2base;
	ifstream readFile(filePath + "\\Data.txt");
	for (int i = 0; i < files.size(); i++)
	{
		/*calibrateHandEye�Ƿ���64F��Ҫͳһ*/
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
		cout << "��е�ֵ�ŷ���ǣ�" << eulerAng[0] <<" "<< eulerAng[1] <<" "<< eulerAng[2] << endl;
		eulerAng = eulerAng * CV_PI / 180.0;

		if (fail_imgs.count(i) != 0)
			continue;

		//ŷ����ת��Ϊ��ת����
		rmatrix = eulerAnglesToRotationMatrix(eulerAng);
		R_gripper2base.push_back(rmatrix);
		cout << "��е�ֵ���ת����\n" << rmatrix<<" "<<isRotationMatrix(rmatrix)<<endl;
		cout << "��е�ֵ�ƽ������Ϊ��" << tvecs.at<double>(0, 0) << " " << tvecs.at<double>(1, 0) << " " << tvecs.at<double>(2, 0) << endl << endl;
		Mat rotationnew;
		Vec3f  e = rotationMatrixToEulerAngles(rmatrix);
		cout << "�µ�ŷ����" << e << endl;
		t_gripper2base.push_back(tvecs);
	}
	readFile.close();
	/**/
	Mat R_target2gripper;
	Mat t_target2gripper;
	/* ����ת����ת��Ϊ���Ӧ����ת���� */
	for(int i = 0; i < R_cam2target.size();i++)
	{
		Rodrigues(R_cam2target[i], R_cam2target[i]);
		cout << "��" << i << "���궨�嵽�������ת����" << R_cam2target[i] << endl;
		Mat Homo = R_T2HomogeneousMatrix(R_cam2target[i], t_cam2target[i]);
		invert(Homo, Homo);
		HomogeneousMtr2RT(Homo, R_cam2target[i], t_cam2target[i]);
		cout << "��" << i << "��������궨�����ת����" << R_cam2target[i] << endl;
		/*����calibrateHandEye����eye in hand�ģ�������������eye to hand����Ҫ��һ���档ͬʱƽ������Ҫȡ������ */
		/*��ʵ��Ҳ��֪��calibratecamera������ı궨������֮�����ת��ϵ��˭��˭�ģ����������涼����һ�£����������ʱ��Ƚ���ȷ*/
		//invert(R_cam2target[i], R_cam2target[i], DECOMP_LU);
		//t_cam2target[i] = t_cam2target[i] * -1;

	//	Vec3f e = rotationMatrixToEulerAngles(R_cam2target[i]);
	//	cout << t_cam2target[i].at<double>(0, 0) << "," << t_cam2target[i].at<double>(1, 0) << "," << t_cam2target[i].at<double>(2, 0) << ","
	//		<< e[0] << "," << e[1] << "," << e[2] <<","<< endl;

	}
	//��AX=XB
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
	/*�����Ӧ����*/
	vector<Mat> Homo_gripper2base, Homo_cam2target;
	Mat  Homo_target2gripper;
	Homo_target2gripper = R_T2HomogeneousMatrix(R_target2gripper, t_target2gripper);
	for (int i = 0; i < image_count; i++)
	{
		Homo_gripper2base.push_back(R_T2HomogeneousMatrix(R_gripper2base[i], t_gripper2base[i]));
		Homo_cam2target.push_back(R_T2HomogeneousMatrix(R_cam2target[i], t_cam2target[i]));

		cout << "ͼ��"<<i<<"����ת�����ƽ������" << endl;
		cout << R_cam2target[i] << t_cam2target[i] << endl;
	}
	/*����*/
	/*�궨�嵽��е�ֵ���ת�����ƽ������*/
	cout << "�궨�嵽��е�ֵĵ�Ӧ����" << endl;
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

		//cout << "��" << i << "��������궨��ĵ�Ӧ����" << endl;
		//cout << Homo_cam2target[i] << endl;
		//cout << "��" << i << "��ץ�ֵ������ĵ�Ӧ����" << endl;
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
	FileStorage fs2(xmlResult2, FileStorage::WRITE); //����XML�ļ�  
	fs2 << "Homo_cam2base" << Homo_cam2basesum;
	fs2.release();
 
	////�Ա궨�����������
	//string txtResult = filePath + "\\calibrateImage\\caliberation_result.txt";
	//ofstream fout(txtResult);  /* ����궨������ļ� */
 
	//double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	//double err = 0.0; /* ÿ��ͼ���ƽ����� */
	//vector<Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	//cout << "\tÿ��ͼ��ı궨��\n";
	//fout << "ÿ��ͼ��ı궨��\n";
	//for (i = 0; i<image_count; i++)
	//{
	//	vector<Point3f> tempPointSet = object_points[i];
	//	/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
	//	projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
	//	/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
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
	//	cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	//	fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	//}
 
	//cout << "����ƽ����" << total_err / image_count << "����" << endl;
	//fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
 
	////���涨����      
	//cout << "��ʼ���涨����������������" << endl;
	//cv::Mat rotation_matrix;
	//fout << "����ڲ�������" << endl;
	//fout << cameraMatrix << endl << endl;
	//fout << "����ϵ����\n";
	//fout << distCoeffs << endl << endl << endl;
	//cout << distCoeffs.cols << endl;
	//for (int i = 0; i<image_count; i++)
	//{
	//	Rodrigues(rvecsMat[i], rotation_matrix);
	//	fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
	//	fout << rvecsMat[i] << endl;
	//	fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
	//	fout << rotation_matrix<< endl;
	//	fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
	//	fout << tvecsMat[i] << endl << endl;
	//}
	//fout << endl;
 
	////��������ڲ�������ͻ���ϵ����������뵽xml
	//cout << "��ʼ��������ڲ�������ͻ���ϵ��������������" << endl;
	//string xmlResult = filePath + "\\calibrateImage\\caliberation_camera.xml";
	//FileStorage fs(xmlResult, FileStorage::WRITE); //����XML�ļ�  
	//fs << "zConst" << 100.0;
	//fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
	//fs.release();
	////����ƽ�ƾ������ת�����s��xml
	//string xml2Result = filePath + "\\calibrateImage\\solvePnP_camera.xml";
	//FileStorage fs2(xml2Result, FileStorage::WRITE); //����XML�ļ� 
	//double s = 100.0;
	//fs2 << "s" << s;
	//fs2 << "rotation_matrix" << rotation_matrix << "tvecsMat" << tvecsMat[0];
	//fs2.release();
	//cout << "�������" << endl;
 
	////�������ͼ��
	//Mat mapx = Mat(image_size, CV_32FC1);
	//Mat mapy = Mat(image_size, CV_32FC1);
	//Mat R = Mat::eye(3, 3, CV_32F);
	//cout << "�������ͼ��" << endl;
	//initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);

	//cout << mapx.at<float>(0, 0)<< endl;
	//cout << mapy.at<float>(0, 0) << endl;
	//for (int i = 0; i != image_count; i++)
	//{
	//	cout << "Frame #" << i + 1 << "..." << endl;
	//	Mat imageSource = imread(files[i]);
	//	Mat newimage = imageSource.clone();
	//	//��һ�ֲ���Ҫת������ķ�ʽ  
	//	undistort(imageSource,newimage,cameraMatrix,distCoeffs);  
	//	//remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);//Ч�ʸ���
	//	string imageFilePath = calibrateDir + "\\" + imageFilesName[i] +"_d"+ ".png";
	//	imwrite(imageFilePath, newimage);
	//}
	//cout << "�������" << endl;
	//string depth_dir = "D:\\Code\\c++\\Azure Kinect\\imgs\\img0_depthcolor.png";
	//Mat depth = imread(depth_dir);
	//cout << depth.rows << " " << depth.cols << endl;
	//Mat depth_new = depth.clone();
	//undistort(depth,depth_new,cameraMatrix,distCoeffs);  
	//imwrite("test.png", depth_new);

 
	//cout << "ȫ����������" << endl;
	int key = waitKey(0);
	return;
}