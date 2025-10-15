#pragma once

#include "perception/vision/camera/CombinedCamera.hpp"
#include "perception/vision/Vision.hpp"

#include "utils/Timer.hpp"
#include "blackboard/Adapter.hpp"
#include "blackboard/Blackboard.hpp"

// Forward declarations
class CameraImageDumper;

class VisionAdapter : Adapter {
friend class AppAdaptor;
public:

    /**
     * Constructor for VisionAdapter which bridges Vision and VisionBlackboard
     */
    VisionAdapter(Blackboard *bb);

    /**
     * Destructor for VisionAdapter
     */
    ~VisionAdapter();

    /**
     * tick Called to get and process each frame
     */
    void tick();

    // TODO: Temporary fix please resolve
	CombinedCamera *combined_camera_;

private:
    // boost::shared_ptr<CombinedFrame> combined_frame_;
    Vision vision_;

    // If debugging, dump camera image(s)
    CameraImageDumper* imageDumper;

    /* Retrieve camera frame */
    void tickCamera();
    
    /* Tick vision processing algorithms */
    void tickProcess();
};
