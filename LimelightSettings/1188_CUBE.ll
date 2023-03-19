//
// Inputs
//
Inputs
{
	Mat source0;
}

//
// Variables
//
Outputs
{
	Mat cvResizeOutput;
	Mat hsvThresholdOutput;
	Mat cvErodeOutput;
	BlobsReport findBlobsOutput;
}

//
// Steps
//

Step CV_resize0
{
    Mat cvResizeSrc = source0;
    Size cvResizeDsize = (0, 0);
    Double cvResizeFx = 1.0;
    Double cvResizeFy = 1.0;
    InterpolationFlagsEnum cvResizeInterpolation = INTER_LINEAR;

    cvResize(cvResizeSrc, cvResizeDsize, cvResizeFx, cvResizeFy, cvResizeInterpolation, cvResizeOutput);
}

Step HSV_Threshold0
{
    Mat hsvThresholdInput = cvResizeOutput;
    List hsvThresholdHue = [110.07194244604317, 167.7133105802048];
    List hsvThresholdSaturation = [90.9597230068716, 253.7024616921289];
    List hsvThresholdValue = [68.79496402877697, 246.29692832764508];

    hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
}

Step CV_erode0
{
    Mat cvErodeSrc = hsvThresholdOutput;
    Mat cvErodeKernel;
    Point cvErodeAnchor = (-1, -1);
    Double cvErodeIterations = 1.0;
    BorderType cvErodeBordertype = BORDER_CONSTANT;
    Scalar cvErodeBordervalue = (-1);

    cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);
}

Step Find_Blobs0
{
    Mat findBlobsInput = cvErodeOutput;
    Double findBlobsMinArea = 150.0;
    List findBlobsCircularity = [0.0, 1.0];
    Boolean findBlobsDarkBlobs = false;

    findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);
}




