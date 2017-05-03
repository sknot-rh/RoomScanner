#include "application.h"
#include "../build/ui_application.h"


parameters* parameters::instance = 0;

/** \brief Constructor and initializing gui
  * \param parent
  */
RoomScanner::RoomScanner (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::RoomScanner) {
    ui->setupUi (this);
    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    this->setWindowTitle ("RoomScanner");
    this->setWindowIcon(QIcon(":/images/Terminator.jpg"));
    movie = new QMovie(":/images/box.gif");

    // Timer for cloud & UI update
    tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(drawFrame()));

    //Create empty clouds
    kinectCloud.reset(new PointCloudT);
    key_cloud.reset(new PointCloudAT);

    //Tell to sensor in which position is expected input
    parameters* params = parameters::GetInstance();
    params->m = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(0.0f,  Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ());
    kinectCloud->sensor_orientation_ = params->m;
    key_cloud->sensor_orientation_ = params->m;

    copying = stream = false;
    sensorConnected = false;
    registered = false;

    try {
        //OpenNIGrabber
        interface = new pcl::OpenNIGrabber();
        sensorConnected = true;
    }
    catch (pcl::IOException e) {
        PCL_INFO("No sensor connected!\n");
        sensorConnected = false;
    }

    //Setting up UI
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->registerKeyboardCallback (&RoomScanner::keyboardEventOccurred, *this);

    ui->qvtkWidget->update ();

    meshViewer.reset (new pcl::visualization::PCLVisualizer ("meshViewer", false));

    ui->qvtkWidget_2->SetRenderWindow (meshViewer->getRenderWindow ());
    meshViewer->setupInteractor (ui->qvtkWidget_2->GetInteractor (), ui->qvtkWidget_2->GetRenderWindow ());
    ui->qvtkWidget_2->update ();

    //Create callback for openni grabber
    if (sensorConnected) {
        boost::function<void (const PointCloudAT::ConstPtr&)> f = boost::bind (&RoomScanner::cloud_cb_, this, _1);
        interface->registerCallback(f);
        interface->start ();
        tmrTimer->start(20); // msec
    }

    stream = true;

    //Connect reset button
    connect(ui->pushButton_reset, SIGNAL (clicked ()), this, SLOT (resetButtonPressed ()));

    //Connect save button
    connect(ui->pushButton_save, SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));

    //Connect poly button
    connect(ui->pushButton_poly, SIGNAL (clicked ()), this, SLOT (polyButtonPressed ()));

    //Connect reg button
    connect(ui->pushButton_reg, SIGNAL (clicked ()), this, SLOT (regButtonPressed ()));

    //Connect save model button
    connect(ui->pushButton_SaveModel, SIGNAL (clicked ()), this, SLOT (saveModelButtonPressed ()));

    //Connect menu checkbox - coord system
    connect(ui->actionShow_Coordinate_System, SIGNAL(triggered(bool)), this, SLOT(coordSysToggled(bool)));

    //Connect menu checkbox - show last frame
    connect(ui->actionShow_captured_frames, SIGNAL(triggered()), this, SLOT(lastFrameToggled()));

    //Connect load action
    connect(ui->actionLoad_Point_Cloud, SIGNAL (triggered()), this, SLOT (loadActionPressed ()));

    //Connect clear action
    connect(ui->actionClear, SIGNAL (triggered()), this, SLOT (actionClearTriggered ()));

    //Connect tab change action
    connect(ui->tabWidget, SIGNAL (currentChanged(int)), this, SLOT (tabChangedEvent(int)));

    //Connect keypoint action
    connect(ui->actionShow_keypoints, SIGNAL(triggered()), this, SLOT(keypointsToggled()));

    //Connect stream button
    connect(ui->pushButton_stream, SIGNAL (clicked ()), this, SLOT (streamButtonPressed ()));

    //Connect smooth action
    connect(ui->actionSmooth_cloud, SIGNAL (triggered()), this, SLOT (actionSmoothTriggered ()));

    //Connect config button
    connect(ui->pushButton_config, SIGNAL (clicked()), this, SLOT (refreshParams ()));

    //Connect quit action
    connect(ui->actionQuit, SIGNAL (triggered()), this, SLOT (actionQuitTriggered ()));

    //Connect closing labels
    connect(this, SIGNAL(closeLabelSignal(int)), this, SLOT(closeLabelSlot(int)));

    //Connect reseting camera
    connect(this, SIGNAL(resetCameraSignal()), this, SLOT(resetCameraSlot()));

    //Add empty pointclouds
    viewer->addPointCloud(kinectCloud, "kinectCloud");

    //viewer->addPointCloud(key_cloud, "keypoints");

    //viewer->setBackgroundColor(0.5f, 0.5f, 0.5f);
    ui->tabWidget->setCurrentIndex(0);

    //load config
    loadConfigFile();

}

/** \brief renders frame with sift keypoints if desired
  */
void RoomScanner::drawFrame() {
    if (stream) {
        if (mtx_.try_lock()) {
            kinectCloud->clear();
            kinectCloud->width = cloudWidth;
            kinectCloud->height = cloudHeight;
            kinectCloud->points.resize(cloudHeight*cloudWidth);
            kinectCloud->is_dense = false;
            // Fill cloud
            float *pX = &cloudX[0];
            float *pY = &cloudY[0];
            float *pZ = &cloudZ[0];
            unsigned long *pRGB = &cloudRGB[0];
            for(int i = 0; i < kinectCloud->points.size(); i++,pX++,pY++,pZ++,pRGB++) {
                kinectCloud->points[i].x = (*pX);
                kinectCloud->points[i].y = (*pY);
                kinectCloud->points[i].z = (*pZ);
                kinectCloud->points[i].rgba = (*pRGB);
                //cloud->points[i].a = 128; //for better stitching?
            }
            mtx_.unlock();
        }

        if (ui->actionShow_keypoints->isChecked() == true) {
            parameters* params = parameters::GetInstance();
            // downsample data for faster computation
            PointCloudT::Ptr tmp;
            tmp.reset(new PointCloudT);
            filters::downsample(kinectCloud, *tmp, 0.05);

            // estimate the sift interest points
            pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
            pcl::PointCloud<pcl::PointWithScale> result;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
            sift.setSearchMethod(tree);
            sift.setScales(params->SIFTmin_scale, params->SIFTn_octaves, params->SIFTn_scales_per_octave);
            sift.setMinimumContrast(params->SIFTmin_contrast);
            sift.setInputCloud(tmp);
            sift.compute(result);

            copyPointCloud(result, *key_cloud); // from PointWithScale to PointCloudAT

            for (int var = 0; var < key_cloud->size(); ++var) {
                key_cloud->points[var].r = 0;
                key_cloud->points[var].g = 255;
                key_cloud->points[var].b = 0;
            }
            viewer->updatePointCloud(key_cloud,"keypoints");
        }
        viewer->updatePointCloud(kinectCloud,"kinectCloud");
        emit(resetCameraSignal());
    }
}

/** \brief callback function to get data from sensor using openni grabber
  * \param ncloud pointer to cloud from sensor
  */
void RoomScanner::cloud_cb_ (const PointCloudAT::ConstPtr &ncloud) {
    if (stream) {
        if (mtx_.try_lock()) {

            // Size of cloud
            cloudWidth = ncloud->width;
            cloudHeight = ncloud->height;

            // resize the XYZ and RGB point vector
            size_t newSize = ncloud->height*ncloud->width;
            cloudX.resize(newSize);
            cloudY.resize(newSize);
            cloudZ.resize(newSize);
            cloudRGB.resize(newSize);

            // assign pointers to copy data
            float *pX = &cloudX[0];
            float *pY = &cloudY[0];
            float *pZ = &cloudZ[0];
            unsigned long *pRGB = &cloudRGB[0];

            // copy data (using pcl::copyPointCloud, the color stream jitters!!! Why?)
            //pcl::copyPointCloud(*ncloud, *cloud);
            for (int j = 0; j<ncloud->height; j++) {
                for (int i = 0; i<ncloud->width; i++,pX++,pY++,pZ++,pRGB++) {
                    PointAT P = ncloud->at(i,j);
                    (*pX) = P.x;
                    (*pY) = P.y;
                    (*pZ) = P.z;
                    (*pRGB) = P.rgba;
                }
            }
            // data copied
            mtx_.unlock();
        }
    }
}

/** \brief renders coordination system axes
  * \param value
  */
void RoomScanner::coordSysToggled(bool value) {
    if (value) {
        viewer->addCoordinateSystem(1, 0, 0, 0, "viewer", 0);
    }
    else {
        viewer->removeCoordinateSystem("viewer", 0);
    }
    ui->qvtkWidget->update();
}

void RoomScanner::resetCameraSlot() {
    viewer->resetCamera();
    ui->qvtkWidget->update();
    meshViewer->resetCamera();
    ui->qvtkWidget_2->update();
}

/** \brief reset camera
  */
void RoomScanner::resetButtonPressed() {
    viewer->resetCamera();
    ui->qvtkWidget->update();
}

/** \brief run loading screen and runs second thread
  */
void RoomScanner::saveButtonPressed() {
    if (!sensorConnected) {
        PCL_INFO("Nothing to save.\n");
        return;
    }
    boost::thread* thr2 = new boost::thread(boost::bind(&RoomScanner::saveButtonPressedFun, this));
    labelSave = new clickLabel(thr2);
    loading(labelSave);
}

/** \brief save current frame from sensor
  */
void RoomScanner::saveButtonPressedFun() {
    PointCloudT::Ptr tmp (new PointCloudT);
    PointCloudT::Ptr output (new PointCloudT);
    stream = false; // "safe" copy
    // allocate enough space and copy the basics
    tmp->header   = kinectCloud->header;
    tmp->width    = kinectCloud->width;
    tmp->height   = kinectCloud->height;
    tmp->is_dense = kinectCloud->is_dense;
    tmp->sensor_orientation_ = kinectCloud->sensor_orientation_;
    tmp->sensor_origin_ = kinectCloud->sensor_origin_;
    tmp->points.resize (kinectCloud->points.size ());

    memcpy (&tmp->points[0], &kinectCloud->points[0], kinectCloud->points.size () * sizeof (PointT));

    // create string for file name
    PCL_INFO("Saving frame #%d\n", clouds.size());
    std::stringstream ss;
    ss << "frame_" << clouds.size()<<  ".pcd";
    std::string s = ss.str();

    //save raw frame
    pcl::io::savePCDFile (s, *tmp);

    pcl::PCLImage::Ptr image (new pcl::PCLImage());
    pcl::io::PointCloudImageExtractorFromRGBField<PointT> pcie;
    pcie.setPaintNaNsWithBlack (true);
    pcie.extract(*tmp, *image);

    //save texture file
    std::stringstream ss2;
    ss2 << "frame_" << images.size()<<  ".png";
    s = ss2.str();
    //pcl::io::savePNGFile(s, *tmp, "rgb");
    pcl::io::savePNGFile(s, *image);
    images.push_back(s);

    // perform filtering
    filters::cloudSmoothFBF(tmp, output);
    std::vector<int> indices;
    removeNaNFromPointCloud(*output,*output, indices);
    filters::oultlierRemoval(output, output, 0.8f);
    clouds.push_back(output);
    lastFrameToggled();
    stream = true;
    //labelSave->close();
    emit(closeLabelSignal(LSAV));
}


/** \brief runs second thread for reading PC from file
 * but file picker is gui element and it does not work well in new thread
  */
/*void RoomScanner::loadActionPressed() {
    boost::thread* thr = new boost::thread(boost::bind(&RoomScanner::loadActionPressedFun, this));
    labelLoad = new clickLabel(thr);
    loading(labelLoad);
}
*/

/** \brief load point cloud from file
  */
void RoomScanner::loadActionPressed() {
    parameters* params = parameters::GetInstance();
    viewer->removeAllPointClouds();
    ui->tabWidget->setCurrentIndex(0);
    QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Choose Point Cloud Files"),QDir::currentPath(), tr("Point Cloud Files (*.pcd)") );

    if( !fileNames.isEmpty() )
    {
        for (int i = 0; i < fileNames.count(); i++) {
            std::string utf8_fileName = fileNames.at(i).toUtf8().constData();
            PointCloudT::Ptr cloudFromFile (new PointCloudT);
            if (pcl::io::loadPCDFile<PointT> (utf8_fileName, *cloudFromFile) == -1) // load the file
            {
                PCL_ERROR ("Couldn't read pcd file!\n");
                return;
            }
            for (size_t i = 0; i < cloudFromFile->size(); i++)
            {
                cloudFromFile->points[i].a = 255;
                if (cloudFromFile->points[i].r == cloudFromFile->points[i].g && cloudFromFile->points[i].g == cloudFromFile->points[i].b && cloudFromFile->points[i].b == 0 ) {
                    cloudFromFile->points[i].r = 127;
                    cloudFromFile->points[i].g = 127;
                    cloudFromFile->points[i].b = 127;
                }

            }
            PCL_INFO("PC Loaded from file %s. Points %d\n", utf8_fileName.c_str(), cloudFromFile->points.size());

            cloudFromFile->sensor_orientation_ = params->m;
            filters::cloudSmoothFBF(cloudFromFile, cloudFromFile);
            viewer->removeAllPointClouds();
            viewer->addPointCloud(cloudFromFile,"cloudFromFile");
            clouds.push_back(cloudFromFile); 
            //this is some weird bug with multithreading and refreshing gui
            //ui->qvtkWidget->update();
            //viewer->resetCamera();
            emit(resetCameraSignal());
        }
    }
    //emit(closeLabelSignal(LLOA));
}

/** \brief runs loading screen and runs second thread
  */
void RoomScanner::polyButtonPressed() {

    if (clouds.empty()) {
        if (sensorConnected) {
            ui->tabWidget->setCurrentIndex(1);
            boost::thread* thr2 = new boost::thread(boost::bind(&RoomScanner::polyButtonPressedFunc, this));
            labelPolygonate = new clickLabel(thr2);
            loading(labelPolygonate);
        }
        else {
            // empty clouds & no sensor
            QMessageBox::warning(this, "Error", "No pointcloud to polygonate!");
            PCL_INFO("No cloud to polygonate!\n");
            return;
        }
    }
    else {
        if (!registered && clouds.size() > 1) {
            QMessageBox::warning(this, "Warning", "Pointclouds ready to registrate!");
            PCL_INFO("Pointclouds ready to registrate!\n");
            return;
        }
        else {
            ui->tabWidget->setCurrentIndex(1);
            boost::thread* thr2 = new boost::thread(boost::bind(&RoomScanner::polyButtonPressedFunc, this));
            labelPolygonate = new clickLabel(thr2);
            loading(labelPolygonate);
        }
    }
    ui->qvtkWidget_2->update();
}

/** \brief loading screen
  */
void RoomScanner::loading(clickLabel* label) {
    if (!movie->isValid()) {
        PCL_INFO("Invalid loading image %s\n", movie->fileName().toStdString());
        return;
    }
    label->setMovie(movie);
    label->setFixedWidth(200);
    label->setFixedHeight(200);
    label->setFrameStyle(QFrame::NoFrame);
    label->setAttribute(Qt::WA_TranslucentBackground);
    label->setWindowModality(Qt::ApplicationModal);
    label->setContentsMargins(0,0,0,0);
    label->setAlignment(Qt::AlignCenter);
    label->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    label->show();
    movie->start();
}

/** \brief determines what and polygonates it
  */
void RoomScanner::polyButtonPressedFunc() {
    PointCloudT::Ptr cloudtmp (new PointCloudT);
    PointCloudT::Ptr output (new PointCloudT);
    PointCloudT::Ptr holder (new PointCloudT);
    triangles.reset(new pcl::PolygonMesh);

    pcl::console::TicToc tt;
    tt.tic();

    //                                                     /+++ polygonate kinect frame
    //                               /+++ sensor connected?
    // polygonation --- empty clouds?                      \--- error           /+++ polygonate cloud
    //                               \--- non empty clouds --- registered cloud?
    //                                                                          \--- register & polygonate?
    //

    if (clouds.empty()) {
        if (sensorConnected) {

            cloudtmp->clear();
            //keep point cloud organized
            cloudtmp->width = cloudWidth;
            cloudtmp->height = cloudHeight;
            cloudtmp->points.resize(cloudHeight*cloudWidth);
            cloudtmp->is_dense = false;
            // Fill cloud
            float *pX = &cloudX[0];
            float *pY = &cloudY[0];
            float *pZ = &cloudZ[0];
            unsigned long *pRGB = &cloudRGB[0];

            for(int i = 0; i < kinectCloud->points.size(); i++,pX++,pY++,pZ++,pRGB++) {
                cloudtmp->points[i].x = (*pX);
                cloudtmp->points[i].y = (*pY);
                cloudtmp->points[i].z = (*pZ);
                cloudtmp->points[i].rgba = (*pRGB);
            }

            PCL_INFO("Empty clouds & sensor connected\n");

            filters::cloudSmoothFBF(cloudtmp, output);
            //filters::bilatelarUpsampling(cloudtmp, output);
            filters::voxelGridFilter(output, output, 0.02);

            if (ui->radioButton_GT->isChecked()) {
                mesh::polygonateCloudGreedyProj(output, triangles);
            }
            else if (ui->radioButton_GP->isChecked()){
                mesh::polygonateCloudGridProj(output, triangles);
            }
            else {
                mesh::polygonateCloudPoisson(output, triangles);
            }

        }
        else {
            PCL_INFO("Empty clouds & sensor disconnected\n");
            QMessageBox::warning(this, "Error", "No pointcloud to polygonate!");
            PCL_INFO("No cloud to polygonate!\n");
            return;
        }
    }
    else {
        if (registered) {
            PCL_INFO("Registered clouds to polygonate\n");
            if (ui->radioButton_GT->isChecked()) {
                mesh::polygonateCloudGreedyProj(regResult, triangles);
            }
            else if (ui->radioButton_GP->isChecked()){
                mesh::polygonateCloudGridProj(regResult, triangles);
            }
            else {
                mesh::polygonateCloudPoisson(regResult, triangles);
            }
        }
        else {
            PCL_INFO("Cloud to polygonate\n");
            filters::cloudSmoothFBF(clouds.back(), clouds.back());
            if (ui->radioButton_GT->isChecked()) {
                mesh::polygonateCloudGreedyProj(clouds.back(), triangles);
            }
            else if (ui->radioButton_GP->isChecked()){
                mesh::polygonateCloudGridProj(clouds.back(), triangles);
            }
            else {
                mesh::polygonateCloudPoisson(clouds.back(), triangles);
            }
        }
    }

    meshViewer->removePolygonMesh("mesh");

    if (ui->groupBox_6->isChecked()) {
        // Hole Filling
        pcl::PolygonMesh::Ptr trianglesFilled(new pcl::PolygonMesh);
        mesh::fillHoles(triangles, trianglesFilled);
        PCL_INFO("After holefilling: %d\n", trianglesFilled->polygons.size());
        triangles = trianglesFilled;
    }

    if (ui->groupBox_7->isChecked()) {
        pcl::PolygonMesh::Ptr trianglesDecimated(new pcl::PolygonMesh);
        mesh::meshDecimation(triangles, trianglesDecimated);
        triangles = trianglesDecimated;
    }

    PCL_INFO("Reconstruction took %g ms\n",tt.toc());
    // Smoothing mesh
    // not sure if necessary, surface is already smooth. This'd just decimate another details
    // mesh::smoothMesh(triangles, triangles);

    /* // Shading setting
    //we want to set phong (or another than flat shading) to our mesh, but... https://github.com/PointCloudLibrary/pcl/issues/178
    meshViewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, "mesh" );
    */

    meshViewer->addPolygonMesh(*triangles, "mesh");
    ui->qvtkWidget_2->update();

    emit(closeLabelSignal(LPOL));
    //QMetaObject::invokeMethod(this, "closeLabelSlot", Qt::BlockingQueuedConnection, Q_ARG(<clickLabel*>, labelPolygonate));
    //labelPolygonate->close();

    PCL_INFO("Mesh done\n");
    //meshViewer->resetCamera();
    emit(resetCameraSignal());

}

/** \brief show last captured frame switch
  */
void RoomScanner::lastFrameToggled() {
    if (clouds.empty()) {
        return;
    }
    if (ui->actionShow_captured_frames->isChecked()) {
        viewer->removePointCloud("frame" + std::to_string(clouds.size()-1));
        for (size_t i = 0; i < clouds.back()->points.size(); i ++) {
            clouds.back()->points[i].a = 50;
        }
        viewer->addPointCloud(clouds.back(), "frame" + std::to_string(clouds.size()));
        //TODO move camera regarding to position in real world, if is it possible
        ui->qvtkWidget->update ();
    }
    else {
        viewer->removePointCloud("frame" + std::to_string(clouds.size()));
        ui->qvtkWidget->update ();
    }
}

/** \brief clears all vectors and viewports
  */
void RoomScanner::actionClearTriggered()
{
    clouds.clear();
    images.clear();
    viewer->removeAllPointClouds();
    meshViewer->removeAllPointClouds();
    ui->qvtkWidget->update();
    ui->qvtkWidget_2->update();
    if (sensorConnected) {
        viewer->addPointCloud(kinectCloud, "kinectCloud");
    }
    ui->tabWidget->setCurrentIndex(0);
    stream = true;
    registered = false;
}

/** \brief runs loading screen and econd thread
  */
void RoomScanner::regButtonPressed() {
    if (clouds.size() < 2) {
        QMessageBox::warning(this, "Error", "To few clouds to registrate!");
        PCL_INFO("To few clouds to registrate!\n");
        return;
    }
    stream = false;
    PCL_INFO("Registrating %d point clouds.\n", clouds.size());
    boost::thread* thr = new boost::thread(boost::bind(&RoomScanner::registerNClouds, this));

    labelRegister = new clickLabel(thr);
    loading(labelRegister);
    //thr->join();
    //labelRegister->close();
}

/** \brief runs registration of frames saved in clouds vector
  */
void RoomScanner::registerNClouds() {
    regResult.reset(new PointCloudT);
    PointCloudT::Ptr source, target;

    registration reg;
    connect(&reg, SIGNAL(regFrameSignal()), this, SLOT(regFrameSlot()));

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
    Eigen::Matrix4f pairTransform1 = Eigen::Matrix4f::Identity ();
    Eigen::Matrix4f pairTransform2 = Eigen::Matrix4f::Identity ();
    viewer->removeAllPointClouds();
    // size of vector check is performed before
    viewer->addPointCloud(clouds[0], "target");
    viewer->addPointCloud(clouds[1], "source");

    if (texturing::stitchImages(images)) {
        PCL_INFO("Texture created in file texture.jpg\n");

    }
    else {
        PCL_INFO("Sorry, no texture\n");
    }

    pcl::console::TicToc tt;
    tt.tic();

    regResult = clouds[0]; //target 1
    for (int i = 1; i < clouds.size(); i++) {
        source = clouds[i];
        PCL_INFO ("source %d\n", source->points.size());
        target = clouds[i-1];
        pcl::transformPointCloud (*source, *source, GlobalTransform);
        viewer->updatePointCloud(target, "target");

        // estimated source position done with fpfh features
        if (!reg.computeTransformation(source, target, pairTransform1))  {
            //labelRegister->close();
            emit(closeLabelSignal(LREG));
            QMessageBox::warning(this, "Error", "No keypoints in input cloud! Stopping registration.");
            return;
        }

        PointCloudT::Ptr temp (new PointCloudT);
        //get transformation between two clouds and transformed source
        reg.pairAlign (source, target, temp, pairTransform2, true);
        //pcl::transformPointCloud (*temp, *target, GlobalTransform);
        pcl::copyPointCloud (*temp, *source);
        *regResult += *source;
        filters::voxelGridFilter(regResult, regResult, 0.02);
        ui->qvtkWidget->update();

        //update the global transform
        GlobalTransform = GlobalTransform * pairTransform1 * pairTransform2;

    }
    PCL_INFO("Registration took %g ms\n",tt.toc());

    viewer->removeAllPointClouds();

    //filters::normalFilter(regResult, regResult);
    viewer->addPointCloud(regResult, "result");
    pcl::io::savePCDFileBinary ("registeredOutput.pcd", *regResult);
    PCL_INFO( "Registrated Point Cloud has %d points.\n", regResult->points.size());

    ui->qvtkWidget->update();
    registered = true;
    emit(closeLabelSignal(LREG));
    //labelRegister->close();

}

/** \brief if app is at another than 1st tam, it is reduntant to stream data
  */
void RoomScanner::tabChangedEvent(int tabIndex) {
    if (tabIndex == 0) {
        stream = true;
    }
    else {
        PCL_INFO("Stopping stream...\n");
        stream = false;
    }
}

/** \brief show/hide keypoints in sensor frame
  */
void RoomScanner::keypointsToggled() {
    if (!ui->actionShow_keypoints->isChecked()) {
        viewer->removePointCloud("keypoints");
        ui->qvtkWidget->update();
    }
    else {
        viewer->addPointCloud(key_cloud, "keypoints");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
        ui->qvtkWidget->update();
    }
}

/** \brief slot for updating viewport during registration
  */
void RoomScanner::regFrameSlot() {
    PCL_INFO("Signal receieved\n");
    try {
        //this causes std::length_error and i do not know, how to fix this :( (only with VTK 7.1)
        //viewer->updatePointCloud(registration::regFrame, "source");
        emit(resetCameraSignal());
    }
    catch (const std::length_error& le) {
          return;
    }
    /*ui->qvtkWidget->update();
    viewer->resetCamera();*/

}

/** \brief runs loading screen and second thread
  */
void RoomScanner::streamButtonPressed() {
    stream = true;
    viewer->removeAllPointClouds();
    viewer->addPointCloud(kinectCloud, "kinectCloud");
    emit(resetCameraSignal());
}

void RoomScanner::actionSmoothTriggered() {
    if (clouds.empty()) {
        QMessageBox::warning(this, "Error", "Nothing to smooth!");
        PCL_INFO("Nothing to smooth\n");
    }
    boost::thread* thr2 = new boost::thread(boost::bind(&RoomScanner::smoothAction, this));
    labelSmooth = new clickLabel(thr2);
    loading(labelSmooth);
}

/** \brief smooth input cloud
  */
void RoomScanner::smoothAction() {
    stream = false;
    PCL_INFO("Smoothing input cloud\n");

    pcl::console::TicToc tt;
    tt.tic();

    if (!clouds.empty()) {
        if (registered){
            filters::voxelGridFilter(regResult, regResult, 0.02);
            filters::cloudSmoothMLS(regResult, regResult);
            filters::normalFilter(regResult, regResult);
            viewer->removeAllPointClouds();
            viewer->addPointCloud(regResult, "smoothCloud");
        }
        else {
            filters::voxelGridFilter(clouds.back(), clouds.back(), 0.02);
            filters::cloudSmoothMLS(clouds.back(), clouds.back());
            viewer->removeAllPointClouds();
            viewer->addPointCloud(clouds.back(), "smoothCloud");
        }
    }
    PCL_INFO("Smoothing took %g ms\n",tt.toc());
    emit closeLabelSignal(LSMO);
    //Does not work with vtk7.1
    //viewer->resetCamera();
    //ui->qvtkWidget->update();
    emit(resetCameraSignal());

}

/** \brief load paramters from config file and update gui form
  */
void RoomScanner::loadConfigFile() {
    parameters* params = parameters::GetInstance();
    std::ifstream config_file("config.json");

    if (!config_file.fail()) {
        PCL_INFO("Config file loaded\n");
        using boost::property_tree::ptree;
        ptree pt;
        read_json(config_file, pt);

        for (auto & array_element: pt) {
                PCL_INFO("%s\n", array_element.first.c_str());
            for (auto & property: array_element.second) {
                PCL_INFO(" %s = %s\n", property.first.c_str(), property.second.get_value < std::string > ().c_str());
            }
            PCL_INFO("\n");
        }


        params->VGFleafSize = pt.get<float>("gridFilter.leafSize");

        ui->lineEdit_VGleaf->setText(QString::number(params->VGFleafSize));


        params->MLSpolynomialOrder = pt.get<int>("mls.polynomialOrder");
        params->MLSusePolynomialFit = pt.get<bool>("mls.usePolynomialFit");
        params->MLSsearchRadius = pt.get<double>("mls.searchRadius");
        params->MLSsqrGaussParam = pt.get<double>("mls.sqrGaussParam");
        params->MLSupsamplingRadius = pt.get<double>("mls.upsamplingRadius");
        params->MLSupsamplingStepSize = pt.get<double>("mls.upsamplingStepSize");
        params->MLSdilationIterations = pt.get<int>("mls.dilationIterations");
        params->MLSdilationVoxelSize = pt.get<double>("mls.dilationVoxelSize");
        params->MLScomputeNormals = pt.get<bool>("mls.computeNormals");

        ui->lineEdit_MLSorder->setText(QString::number(params->MLSpolynomialOrder));
        ui->lineEdit_MLSradius->setText(QString::number(params->MLSsearchRadius));
        ui->lineEdit_MLSgauss->setText(QString::number(params->MLSsqrGaussParam));
        ui->lineEdit_MLSupRadius->setText(QString::number(params->MLSupsamplingRadius));
        ui->lineEdit_MLSupSize->setText(QString::number(params->MLSupsamplingStepSize));
        ui->lineEdit_MLSditer->setText(QString::number(params->MLSdilationIterations));
        ui->lineEdit_MLSdvsize->setText(QString::number(params->MLSdilationVoxelSize));
        ui->checkBox_MLSnormals->setChecked(params->MLScomputeNormals);
        ui->checkBox_MLSpolyfit->setChecked(params->MLSusePolynomialFit);


        params->GPsearchRadius = pt.get<double>("greedyProjection.searchRadius");
        params->GPmu = pt.get<double>("greedyProjection.mu");
        params->GPmaximumNearestNeighbors = pt.get<int>("greedyProjection.maximumNearestNeighbors");

        ui->lineEdit_GPserrad->setText(QString::number(params->GPsearchRadius));
        ui->lineEdit_GPmaxneigh->setText(QString::number(params->GPmaximumNearestNeighbors));
        ui->lineEdit_GPmu->setText(QString::number(params->GPmu));


        params->SIFTmin_scale = pt.get<double>("SIFT.min_scale");
        params->SIFTn_octaves = pt.get<int>("SIFT.n_octaves");
        params->SIFTn_scales_per_octave = pt.get<int>("SIFT.n_scales_per_octave");
        params->SIFTmin_contrast = pt.get<double>("SIFT.min_contrast");

        ui->lineEdit_SIFTmin_con->setText(QString::number(params->SIFTmin_contrast));
        ui->lineEdit_SIFTmin_scale->setText(QString::number(params->SIFTmin_scale));
        ui->lineEdit_SIFTn_octaves->setText(QString::number(params->SIFTn_octaves));
        ui->lineEdit_SIFTscales->setText(QString::number(params->SIFTn_scales_per_octave));


        params->REGnormalsRadius = pt.get<double>("registration.normalsRadius");
        params->REGfpfh = pt.get<double>("registration.fpfh");
        params->REGreject = pt.get<double>("registration.reject");
        params->REGcorrDist = pt.get<double>("registration.corrDist");

        ui->lineEdit_REGcorrejdist->setText(QString::number(params->REGreject));
        ui->lineEdit_REGfpfh->setText(QString::number(params->REGfpfh));
        ui->lineEdit_REGmaxCorrDist->setText(QString::number(params->REGcorrDist));
        ui->lineEdit_REGnormals->setText(QString::number(params->REGnormalsRadius));


        params->FBFsigmaS = pt.get<double>("fastBFilter.sigmaS");
        params->FBFsigmaR = pt.get<double>("fastBFilter.sigmaR");

        ui->lineEdit_FBSigmaR->setText(QString::number(params->FBFsigmaR));
        ui->lineEdit_FBSigmaS->setText(QString::number(params->FBFsigmaS));


        params->DECtargetReductionFactor = pt.get<double>("decimation.targetReductionFactor");

        ui->lineEdit_DECfactor->setText(QString::number(params->DECtargetReductionFactor));


        params->HOLsize = pt.get<double>("holeFill.size");

        ui->lineEdit_HOLsize->setText(QString::number(params->HOLsize));


        params->GRres = pt.get<double>("gridProj.size");

        ui->lineEdit_GRres->setText(QString::number(params->GRres));


        params->POSdepth = pt.get<int>("poissonProj.depth");

        ui->lineEdit_POSdepth->setText(QString::number(params->POSdepth));
    }
}

/** \brief updates parameter from gui form
  */
void RoomScanner::refreshParams() {
    parameters* params = parameters::GetInstance();

    params->VGFleafSize = ui->lineEdit_VGleaf->text().toDouble();


    params->MLSpolynomialOrder = ui->lineEdit_MLSorder->text().toInt();
    params->MLSusePolynomialFit = ui->checkBox_MLSpolyfit->isChecked();
    params->MLSsearchRadius = ui->lineEdit_MLSradius->text().toDouble();
    params->MLSsqrGaussParam = ui->lineEdit_MLSgauss->text().toDouble();
    params->MLSupsamplingRadius = ui->lineEdit_MLSupRadius->text().toDouble();
    params->MLSupsamplingStepSize = ui->lineEdit_MLSupSize->text().toDouble();
    params->MLSdilationIterations = ui->lineEdit_MLSditer->text().toInt();
    params->MLSdilationVoxelSize = ui->lineEdit_MLSdvsize->text().toDouble();
    params->MLScomputeNormals = ui->checkBox_MLSnormals->isChecked();


    params->GPsearchRadius = ui->lineEdit_GPserrad->text().toDouble();
    params->GPmu = ui->lineEdit_GPmu->text().toDouble();
    params->GPmaximumNearestNeighbors = ui->lineEdit_GPmaxneigh->text().toInt();


    params->SIFTmin_scale = ui->lineEdit_SIFTmin_scale->text().toDouble();
    params->SIFTn_octaves = ui->lineEdit_SIFTn_octaves->text().toInt();
    params->SIFTn_scales_per_octave = ui->lineEdit_SIFTscales->text().toInt();
    params->SIFTmin_contrast = ui->lineEdit_SIFTmin_con->text().toDouble();


    params->REGnormalsRadius = ui->lineEdit_REGnormals->text().toDouble();
    params->REGfpfh = ui->lineEdit_REGfpfh->text().toDouble();
    params->REGreject = ui->lineEdit_REGcorrejdist->text().toDouble();
    params->REGcorrDist = ui->lineEdit_REGmaxCorrDist->text().toDouble();


    params->FBFsigmaS = ui->lineEdit_FBSigmaS->text().toDouble();
    params->FBFsigmaR = ui->lineEdit_FBSigmaR->text().toDouble();


    params->DECtargetReductionFactor = ui->lineEdit_DECfactor->text().toDouble();


    params->HOLsize = ui->lineEdit_HOLsize->text().toDouble();


    params->GRres = ui->lineEdit_GRres->text().toDouble();


    params->POSdepth = ui->lineEdit_POSdepth->text().toInt();


    PCL_INFO("Parameters refreshed.\n");
    ui->tabWidget->setCurrentIndex(0);
}

/** \brief save output mesh to file
  */
void RoomScanner::saveModelButtonPressed() {
    if (triangles == NULL || triangles->polygons.size() == 0) {
        PCL_INFO("Nothing to save\n");
        return;
    }
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setNameFilter(tr("Polygonal model (*.ply *.obj)"));
    QString fileName;
    if (dialog.exec())
        fileName = dialog.selectedFiles().at(0);
    std::string extension = fileName.split(".",QString::SkipEmptyParts).at(1).toUtf8().constData();
    if (extension.compare("ply") == 0) {
        PCL_INFO("Saving %s\n",  fileName.toUtf8().constData());
        pcl::io::savePLYFile(fileName.toUtf8().constData(), *triangles);
    }
    else if (extension.compare("obj") == 0) {
        PCL_INFO("Saving %s\n",  fileName.toUtf8().constData());
        pcl::io::saveOBJFile(fileName.toUtf8().constData(), *triangles);
    }
    else {
        PCL_INFO("Unsupported format.\n");
        return;
    }
}

/** \brief Quit application, calls destructor
  */
void RoomScanner::actionQuitTriggered() {
    QCoreApplication::quit();
}

/** \brief keyboard event listener
  */
void RoomScanner::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void) {

  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "space" && event.keyDown ())
  {
    RoomScanner::saveButtonPressed();
  }
}

/**
 * \brief close loading screen
 * \param index which label to close
 */
void RoomScanner::closeLabelSlot(int index) {
    switch (index) {
    case 0:
        labelRegister->close();
        break;
    case 1:
        labelPolygonate->close();
        break;
    case 2:
        labelSave->close();
        break;
    case 3:
        labelSmooth->close();
        break;
    case 4:
        labelLoad->close();
        break;
    default:
        break;
    }
}

/** \brief destructor
  */
RoomScanner::~RoomScanner ()
{
    PCL_INFO("Exiting...\n");
    if (sensorConnected) {
        interface->stop();
    }
    delete ui;
    clouds.clear();
    images.clear();
    tmrTimer->stop();
    //delete &cloud;
}
