#include "perception/PerceptionThread.hpp"

#include <pthread.h>
#include <ctime>
#include <utility>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

#include "soccer.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/PerceptionBlackboard.hpp"
#include "blackboard/modules/SynchronisationBlackboard.hpp"
#include "blackboard/modules/ThreadBlackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "perception/dumper/PerceptionDumper.hpp"
#include "perception/behaviour/SafetySkill.hpp"
#include "thread/Thread.hpp"
#include "utils/Logger.hpp"

using namespace std;
using namespace boost;

PerceptionThread::PerceptionThread(Blackboard *bb)
    : Adapter(bb),
      stateEstimationAdapter(bb),
      behaviourAdapter(bb),
      bb_(bb)
{
    visionAdapter = new VisionAdapter(bb);

    std::system("sudo pkill -9 -f $HOME/data/behaviours/audio/whistle_detector.py"); //Close any existing whistle detectors
    std::system("python3 $HOME/data/behaviours/audio/whistle_detector.py &"); //Activate whistle detector

    dumper = NULL;
    // pythonLandmarks = PythonLandmarks();

    releaseLock(serialization);
    uint8_t const *topFrame = readFrom(vision, topFrame);
    uint8_t const *botFrame = readFrom(vision, botFrame);

    if (topFrame != NULL && botFrame != NULL) {
        string file = "/home/nao/crashframe-" +
                    boost::lexical_cast<string>(time(NULL)) + ".yuv";
        FILE *errorFrameFile = fopen(file.c_str(), "w");
        fwrite(topFrame, 640 * 480 * 2, 1, errorFrameFile);
        fwrite(botFrame, 640 * 480 * 2, 1, errorFrameFile);
        fclose(errorFrameFile);
        file =
            "/usr/bin/tail -n 200 " + bb->config["debug.log.dir"].as<string>() + "/*/Perception > " + file + ".log";
        std::system(file.c_str());
    }

    readOptions(bb->config);
    writeTo(thread, configCallbacks[Thread::name],
            boost::function<void(const boost::program_options::variables_map &)>(boost::bind(&PerceptionThread::readOptions, this, _1)));
}

PerceptionThread::~PerceptionThread()
{
    llog(INFO) << __PRETTY_FUNCTION__ << endl;
    writeTo(thread, configCallbacks[Thread::name], boost::function<void(const boost::program_options::variables_map &)>());
    delete visionAdapter;
    std::system("sudo pkill -9 -f $HOME/data/behaviours/audio/whistle_detector.py"); //Close any existing whistle detectors
}

void PerceptionThread::tick()
{
    llog(TRACE) << "Perception Thread:: Tick" << endl;

    Timer timer_thread;
    Timer timer_tick;

    /*
    * Vision Tick
    */
    llog(TRACE) << "Vision Tick" << endl;
    timer_tick.restart();
    if (visionAdapter)
        visionAdapter->tick();

    uint32_t vision_time = timer_tick.elapsed_us();
    if (vision_time < TICK_MAX_TIME_VISION)
    {
        llog(TRACE) << "Vision Tick: OK " << vision_time << endl;
    }
    else
    {
        llog(TRACE) << "Vision Tick: TOO LONG " << vision_time << endl;
    }

    /*
    * State Estimation Tick
    */
    llog(TRACE) << "State Estimation Tick" << endl;
    timer_tick.restart();

    stateEstimationAdapter.tick();

    uint32_t state_estimation_time = timer_tick.elapsed_us();
    if (state_estimation_time < TICK_MAX_TIME_STATE_ESTIMATION)
    {
        llog(TRACE) << "State Estimation Tick: OK " << state_estimation_time << endl;
    }
    else
    {
        llog(TRACE) << "State Estimation Tick: TOO LONG " << state_estimation_time << endl;
    }

    /*
    * Behaviour Tick
    */
    llog(TRACE) << "Behaviour Tick" << endl;
    timer_tick.restart();
    pthread_yield();
    behaviourAdapter.tick();

    // pythonLandmarks.execute();
    

    uint32_t behaviour_time = timer_tick.elapsed_us();
    if (behaviour_time < TICK_MAX_TIME_BEHAVIOUR)
    {
        llog(TRACE) << "Behaviour Tick (and perception yield): OK " << behaviour_time << endl;
    }
    else
    {
        llog(TRACE) << "Behaviour Tick (and perception yield): TOO LONG " << behaviour_time << endl;
    }

    if (!visionAdapter)
        // Introduce delay to compensate for vision processing
        // (so behaviours runs at the correct speed)
        boost::this_thread::sleep(boost::posix_time::milliseconds(32));


    /*
    * Finishing Perception
    */
    uint32_t perception_time = timer_thread.elapsed_us();
    if (perception_time < THREAD_MAX_TIME)
    {
        llog(TRACE) << "Perception Thread: OK " << perception_time << endl;
    }
    else
    {
        llog(TRACE) << "Perception Thread: TOO LONG " << perception_time << endl;
    }

    writeTo(perception, vision, vision_time);
    writeTo(perception, stateEstimation, state_estimation_time);
    writeTo(perception, behaviour, behaviour_time);
    writeTo(perception, total, perception_time);

    if (dumper)
    {
        if (dump_timer.elapsed_us() > dump_rate)
        {
            dump_timer.restart();
            try
            {
                dumper->dump(bb_);
            }
            catch (const std::exception &e)
            {
                attemptingShutdown = true;
                cout << "Error: " << e.what() << endl;
            }
        }
    }
}

void PerceptionThread::readOptions(const boost::program_options::variables_map &config)
{
    if (visionAdapter) {
       const string &e = config["vision.camera_controls"].as<string>();
       vector<string> vs;
       split(vs, e, is_any_of(",;"));
       for (vector<string>::const_iterator ci = vs.begin(); ci != vs.end(); ++ci) {
          vector<string> nv;
          split(nv, *ci, is_any_of(":"));
          if (nv.size() != 3)
             llog(ERROR) << "controls should be cam:control_id:value" << endl;
          else {
             Camera *currCamera;
             if (strtol(nv[0].c_str(), NULL, 10) == 0) {
                currCamera = visionAdapter->combined_camera_->getCameraBot();
             } else {
                currCamera = visionAdapter->combined_camera_->getCameraTop();
             }
             if (strtoul(nv[1].c_str(), NULL, 10) == 0)
                currCamera->setControl(22, 1);
             if (strtoul(nv[1].c_str(), NULL, 10) == 17)
                currCamera->setControl(22, 0);
             if (strtoul(nv[1].c_str(), NULL, 10) == 19)
                currCamera->setControl(22, 0);
             currCamera->setControl(strtoul(nv[1].c_str(), NULL, 10),
                                    strtol(nv[2].c_str(), NULL, 10));
          }
       }
    }

    const string &dumpPath = config["debug.dump"].as<string>();
    dump_rate = config["vision.dumprate"].as<int>() * 1000;
    if (dumpPath == "")
    {
        delete dumper;
        dumper = NULL;
    }
    else
    {
        if (!dumper || dumper->getPath() != dumpPath)
        {
            delete dumper;
            dumper = new PerceptionDumper(dumpPath.c_str());
        }
    }

    OffNaoMask_t dumpMask = config["debug.mask"].as<int>();
    bb_->write(&(bb_->mask), dumpMask);
}
