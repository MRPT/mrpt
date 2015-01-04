static mxArray* from(const cv::Mat& mat)
{
    mxArray* p_; // Create pointer
    if (mat.empty())
    {
        p_ = mxCreateNumericArray(0, 0, mxDOUBLE_CLASS, mxREAL);
        if (!p_)
            mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
        return p_;
    }
    // Optional arguments:
    mxClassID classid = mxUNKNOWN_CLASS;
    bool transpose = true;

    cv::Mat input = (mat.dims == 2 && transpose) ? mat.t() : mat;
    // Create a new mxArray.
    const int nchannels = input.channels();
    const int* dims_ = input.size;
    std::vector<mwSize> d(dims_, dims_ + input.dims);
    d.push_back(nchannels);
    classid = (classid == mxUNKNOWN_CLASS)
        ? ClassIDOf[input.depth()] : classid;
    std::swap(d[0], d[1]);
    if (classid == mxLOGICAL_CLASS)
    {
        // OpenCV's logical true is any nonzero while matlab's true is 1.
        cv::compare(input, 0, input, cv::CMP_NE);
        input.setTo(1, input);
        p_ = mxCreateLogicalArray(d.size(), &d[0]);
    }
    else {
        p_ = mxCreateNumericArray(d.size(), &d[0], classid, mxREAL);
    }
    if (!p_)
        mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
    // Copy each channel.
    std::vector<cv::Mat> channels;
    split(input, channels);
    std::vector<mwSize> si(d.size(), 0); // subscript index.
    int type = CV_MAKETYPE(DepthOf[classid], 1); // destination type.
    for (int i = 0; i < nchannels; ++i)
    {
        si[si.size() - 1] = i; // last dim is a channel index.

        mwIndex subs_si = mxCalcSingleSubscript(p_, si.size(), &si[0]);
//        void *ptr = reinterpret_cast<void*>(
//                reinterpret_cast<size_t>(mxGetData(p_)) +
//                mxGetElementSize(p_) * subs(si));
        void *ptr = reinterpret_cast<void*>(
                reinterpret_cast<size_t>(mxGetData(p_)) +
                mxGetElementSize(p_) * subs_si);
        cv::Mat m(input.dims, dims_, type, ptr);
        //channels[i].convertTo(m, type); // Write to mxArray through m.
        // Swap R and B channels
        MRPT_TODO("Do in other place where it is more clear")
        channels[nchannels-1-i].convertTo(m, type); // Write to mxArray through m.
    }
    return p_;
}
