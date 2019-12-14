//! finds checkerboard pattern of the specified size in the image
// CV_EXPORTS_W bool findChessboardCorners( InputArray image, Size patternSize,
//                                          OutputArray corners,
//                                          int flags=CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE );
// 第一个参数Image，传入拍摄的棋盘图Mat图像，必须是8位的灰度或者彩色图像；
// 第二个参数patternSize，每个棋盘图上内角点的行列数，一般情况下，行列数不要相同，便于后续标定程序识别标定板的方向；
// 第三个参数corners，用于存储检测到的内角点图像坐标位置，一般用元素是Point2f的向量来表示：vector<Point2f> image_points_buf;
// 第四个参数flage：用于定义棋盘图上内角点查找的不同处理方式，有默认值。


// //! adjusts the corner locations with sub-pixel accuracy to maximize the certain cornerness criteria
// CV_EXPORTS_W void cornerSubPix( InputArray image, 【InputOutputArray】 corners,
//                                 Size winSize, Size zeroZone,
//                                 TermCriteria criteria );
//         image	Input single-channel, 8-bit or float image.
//         corners	Initial coordinates of the input corners and refined coordinates provided for output.
//         winSize	【Half】 of the side length of the search window. For example, if winSize=Size(5,5) , then a (5∗2+1)×(5∗2+1)=11×11 search window is used.
//         zeroZone	Half of the size of the dead region in the middle of the search zone over which the summation in the formula below is not done. It is used sometimes to avoid possible singularities of the autocorrelation matrix. The value of (-1,-1) indicates that there is no such a size.
//         criteria	Criteria for termination of the iterative process of corner refinement. That is, the process of corner position refinement stops either after criteria.maxCount iterations or when the corner position moves by less than criteria.epsilon on some iteration.

// CV_EXPORTS_W void drawChessboardCorners( InputOutputArray image, Size patternSize,
//                                          InputArray corners, bool patternWasFound );


// CV_EXPORTS_W double calibrateCamera( InputArrayOfArrays objectPoints,
//                                      InputArrayOfArrays imagePoints,
//                                      Size imageSize,图像的像素尺寸大小
//                                      CV_OUT InputOutputArray cameraMatrix,
//                                      CV_OUT InputOutputArray distCoeffs,
//                                      OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs,
//                                      int flags=0, TermCriteria criteria = TermCriteria(
//                                      TermCriteria::COUNT+TermCriteria::EPS, 30, DBL_EPSILON);


// CV_EXPORTS_W void projectPoints( InputArray objectPoints,
//                                  InputArray rvec, InputArray tvec,
//                                  InputArray cameraMatrix, InputArray distCoeffs,
//                                  OutputArray imagePoints,
//                                  OutputArray jacobian=noArray(),
//                                  double aspectRatio=0 );



// CV_EXPORTS_W void undistort( InputArray src, OutputArray dst,
//                              InputArray cameraMatrix,
//                              InputArray distCoeffs,
//                              InputArray newCameraMatrix=noArray() );

