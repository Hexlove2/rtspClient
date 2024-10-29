/*
Data: 2024/10/14
xyh
*/

// C++
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>
#include <string>
#include <regex>

// live555
#include "BasicUsageEnvironment.hh"
#include "liveMedia.hh"

// ffmpeg
extern "C" {
#include "libavcodec/avcodec.h"
}

// pybind
#include "pySV.hh"

#define maxFrame 5000
#define DATE1 "29"
// client->>server: DESCRIBE
// server->>client: 200 OK (SDP)
// client->>server: SETUP
// server->>client: 200 OK
// client->>server: PLAY
// server->>client: RTP/ client->>server: TERADOWN
struct FrameData {
  int width;
  int height;
  int FPS;
  AVPixelFormat format;
};

struct AudioData{
  int profile;
  int sampleRate;
  int channels;
  bool inc_head;
  bool is_set;
};

std::queue<std::pair<uint8_t *, int>> queue264;
std::queue<AVFrame *> queueFrame;
std::queue<std::pair<uint8_t *, int>> queue265;

std::mutex mutex1;
std::mutex mutex2;
std::mutex mutex3;
std::mutex m_set;

std::condition_variable cv1;
std::condition_variable cv2;
std::condition_variable cv3;
std::condition_variable cv_set;

Boolean is_run1;
Boolean is_run2;
Boolean is_run3;

struct FrameData fd;
struct AudioData ad;
int count264;
std::ostringstream oss;
Boolean set;

void continueAfterDESCRIBE(RTSPClient *rtspClient, int resultCode,
                           char *resultString);
void continueAfterSETUP(RTSPClient *rtspClient, int resultCode,
                        char *resultString);
void continueAfterPLAY(RTSPClient *rtspClient, int resultCode,
                       char *resultString);

void subsessionAfterPlaying(void *clientData);

void subsessionByeHandler(void *clientData, char const *reason);

void streamTimerHandler(void *clientData);

void openURL(UsageEnvironment &env, char const *progName, char const *rtspURL);

void setupNextSubsession(RTSPClient *rtspClient);

void shutdownStream(RTSPClient *rtspClient, int exitCode = 1);

static void thread_decode();

static void thread_codec();

static void thread_save();

static void ad_set(char* descripition);

static void addADTSHeader(uint8_t* adtsHeader, int dataLength, int profile = 2, int sampleRate = 7, int channels = 2);

static bool inc_head(uint8_t* data, int type);

UsageEnvironment &operator<<(UsageEnvironment &env,
                             const RTSPClient &rtspClient) {
  return env << "[URL:\"" << rtspClient.url() << "\"];";
}

UsageEnvironment &operator<<(UsageEnvironment &env,
                             const MediaSubsession &subsession) {
  return env << subsession.mediumName() << "/" << subsession.codecName();
}

void usage(UsageEnvironment &env, char const *progName) {
  env << "Usage: " << progName << "<rtsp-url-1>...<rtsp-url-N \n";
}

// Loop exit if this variable change
char eventLoopWatchVariable = 0;

class StreamClientState {
public:
  StreamClientState();
  virtual ~StreamClientState();

public:
  // Through iter, the client can sequentially access each subsession and
  // perform opeartions like SETUP, PLAY.
  MediaSubsessionIterator *iter;
  MediaSession *session;
  MediaSubsession *subsession;

  // You can use streamTimerTask to set and cancle timer tasks
  TaskToken streamTimerTask;
  double duration;
};

int main(int argc, char **argv) {

  //py_test();
  TaskScheduler *scheduler = BasicTaskScheduler::createNew();
  UsageEnvironment *env = BasicUsageEnvironment::createNew(*scheduler);

  count264 = 0;
  ad.is_set = false;
  // Get current time
  auto now = std::chrono::system_clock::now();
  std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

  // Use stringstream to format the time into a string
  oss << std::put_time(std::localtime(&currentTime), "%H:%M");

  is_run1 = true;
  std::thread decode(thread_decode);
  decode.detach();

  is_run2 = true;
  std::thread encode(thread_codec);
  encode.detach();

  is_run3 = true;
  std::thread save(thread_save);
  save.detach();

  if (argc < 2) {
    usage(*env, argv[0]);
    return 0;
  }
  // Connect to every server
  for (int i = 1; i <= argc - 1; i++) {
    openURL(*env, argv[0], argv[i]);
  }

  env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
  return 0;
}

class ourRTSPClient : public RTSPClient {
public:
  static ourRTSPClient *createNew(UsageEnvironment &env, char const *rtspURL,
                                  int verbosityLevel = 0,
                                  char const *applicationName = NULL,
                                  portNumBits tunnelOverHTTPPortNum = 0);

protected:
  ourRTSPClient(UsageEnvironment &env, char const *rtspURL, int verbosityLevel,
                char const *applicationName, portNumBits tunnelOverHTTPPortNum);
  virtual ~ourRTSPClient();

public:
  StreamClientState scs;
};

class DummySink : public MediaSink {
public:
  static DummySink *createNew(UsageEnvironment &env,
                              MediaSubsession &subsession,
                              char const *streamId = NULL);

private:
  DummySink(UsageEnvironment &env, MediaSubsession &subsession,
            char const *streamId);
  virtual ~DummySink();

  static void afterGettingFrame(void *clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                         struct timeval presentationTime,
                         unsigned durationInMicroseconds);

private:
  virtual Boolean continuePlaying();

private:
  u_int8_t *fReceiveBuffer;
  MediaSubsession &fSubsession;
  char *fStreamId;
  FILE *fOutputFile;
  FILE *aacFile;
};

#define RTSP_CLIENT_VERBOSITY_LEVEL 1

static unsigned rtspClientCount = 0;

void openURL(UsageEnvironment &env, char const *progName, char const *rtspURL) {

  RTSPClient *rtspClient = ourRTSPClient::createNew(
      env, rtspURL, RTSP_CLIENT_VERBOSITY_LEVEL, progName);
  if (rtspClient == NULL) {
    env << "Failed to create a RTSP client for URL \"" << rtspURL
        << "\":" << env.getResultMsg() << "\n";
    return;
  }

  ++rtspClientCount;
  rtspClient->sendDescribeCommand(continueAfterDESCRIBE);
}

// It's a callback function that get triggered when the DESCRIBE response is
// received from the server The DESCRIBE command is used to request a
// description of the media stream in SDP format
void continueAfterDESCRIBE(RTSPClient *rtspClient, int resultCode,
                           char *resultString) {

  do {
    UsageEnvironment &env = rtspClient->envir();
    StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs;

    if (resultCode != 0) {
      env << *rtspClient << "Failed to get a SDP description: " << resultString
          << "\n";
      delete[] resultString;
      break;
    }

    char *const sdpDescription = resultString;
    env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";

    ad_set(sdpDescription);

    scs.session = MediaSession::createNew(env, sdpDescription);
    delete[] sdpDescription;
    if (scs.session == NULL) {
      env << *rtspClient
          << "Failed to create a mediasession object from the SDP description"
          << env.getResultMsg() << "\n";
      break;
    } else if (!scs.session->hasSubsessions()) {
      env << *rtspClient << "This session has no media subsessions \n";
      break;
    }

    // Then, create and set up our data source objects for the session. We do
    // this by iterating over the session's subsessions. calling
    // "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command,
    // on each one (Each 'subsession' will have its own data source)
    scs.iter = new MediaSubsessionIterator(*scs.session);
    setupNextSubsession(rtspClient);

    // SUCESS
    return;
  } while (0);

  // Unknown and Unrecoverable ERROR occured with this stream
  shutdownStream(rtspClient);
}

// By default, we request that the server stream its data using RTP/UDP.
// If, instead, you want to request that the server stream via RTP over TCP,
// changing the following to TRUE. Ensure that the server you wanna connect
// supports RTP/TCP, Otherwise, the RTSP server will most likely reject the
// request during the RTSP SETUP phase.
#define REQUEST_STREAM_OVER_TCP False

// its primary goal is to iterate through all media subsessions within an RTSP
// mediasession, initiate each subsession, and send an RTSP SETUP command for
// each one. After all subsessions are set up, it finally sends a PLAY command
// to start streaming.
void setupNextSubsession(RTSPClient *rtspClient) {

  // Alias
  UsageEnvironment &env = rtspClient->envir();
  StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs;

  scs.subsession = scs.iter->next();
  if (scs.subsession != NULL) {
    if (!scs.subsession->initiate()) {
      env << *rtspClient << "Failed to initiate the\"" << *scs.subsession
          << "\" subsession: " << env.getResultMsg() << "\n";
      setupNextSubsession(
          rtspClient); // give up on this subsession; go to the next one
    } else {
      env << *rtspClient << "Initiated the \"" << *scs.subsession
          << "\" subsession (";
      if (scs.subsession->rtcpIsMuxed()) {
        env << "client port " << scs.subsession->clientPortNum();
      } else {
        env << "client ports " << scs.subsession->clientPortNum() << "-"
            << scs.subsession->clientPortNum() + 1;
      }
      env << "\n";

      // Continue setting up this subsession, by sending a "SETUP" command;
      rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False,
                                   REQUEST_STREAM_OVER_TCP);
    }
    return;
  }

  // We've finished all of the subsession. Now, send a RTSP "PLAY" command to
  // start streaming:
  if (scs.session->absStartTime() != NULL) {
    // Special case: The stream is indexed by 'absolute time', so send a
    // appropriate "PLAY" command
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY,
                                scs.session->absStartTime(),
                                scs.session->absEndTime());
  } else {
    scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
  }
}

// Callback function
void continueAfterSETUP(RTSPClient *rtspClient, int resultCode,
                        char *resultString) {
  do {
    // Alias
    UsageEnvironment &env = rtspClient->envir();
    StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs;

    if (resultCode != 0) {
      env << *rtspClient << "Failed to set up the \"" << *scs.subsession
          << "\" subsession: " << resultString << "\n";
      break;
    }

    env << *rtspClient << "Set up the \"" << *scs.subsession
        << "\" subsession(";
    if (scs.subsession->rtcpIsMuxed()) {
      env << "Client port " << scs.subsession->clientPortNum();
    } else {
      env << "Client ports " << scs.subsession->clientPortNum() << "-"
          << scs.subsession->clientPortNum() + 1;
    }
    env << ")\n";

    // Having successfully setup the subsession, create a data sink for it, and
    // call "startPlaying()" on it. (This will prepare the data sink to receive
    // data; the actual flow from the client won't start happening untill later
    // after we've sent an RTSP "PLAY" command.)

    // Perhaps use your own custom "MediaSink" subclass instead
    scs.subsession->sink =
        DummySink::createNew(env, *scs.subsession, rtspClient->url());

    if (scs.subsession->sink == NULL) {
      env << *rtspClient << "Failed to create a data sink for the \""
          << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
      break;
    }



    // // Create an H264VideoStreamFramer to reassemble fragmented NAL units
    // H264VideoStreamFramer *videoFramer = H264VideoStreamFramer::createNew(env, scs.subsession->rtpSource());

    // env << *rtspClient << "Create an H264VideoStreamFramer for the \"" << *scs.subsession
    //             << "\" subsession\n";

    // scs.subsession->miscPtr = rtspClient;
    // scs.subsession->sink->startPlaying(*videoFramer, subsessionAfterPlaying, scs.subsession);

    // // Set a handler for when the RTSP server sends a "BYE"
    // if (scs.subsession->rtcpInstance() != NULL) {
    //             scs.subsession->rtcpInstance()->setByeWithReasonHandler(subsessionByeHandler, scs.subsession);
    // }



    env << *rtspClient << "Create a data sink for the \"" << *scs.subsession
        << "\" subsession\n";
    scs.subsession->miscPtr = rtspClient;
    scs.subsession->sink->startPlaying(*(scs.subsession->readSource()),
                                       subsessionAfterPlaying, scs.subsession);
    // Also set a handler to be called if a RTSP "BYE" arrives for this
    // subsession
    if (scs.subsession->rtcpInstance() != NULL) {
      scs.subsession->rtcpInstance()->setByeWithReasonHandler(
          subsessionByeHandler, scs.subsession);
    }
  } while (0);
  delete[] resultString;

  // Set up the next subsession, if any:
  setupNextSubsession(rtspClient);
}

// A callback function that is triggered after the RTSP client sends a PLAY
// command and receives a response from the RTSP server
void continueAfterPLAY(RTSPClient *rtspClient, int resultCode,
                       char *resultString) {
  Boolean success = False;

  do {
    // Alias
    UsageEnvironment &env = rtspClient->envir();
    StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs;

    if (resultCode != 0) {
      env << *rtspClient << "Failed to start playing session: " << resultString
          << "\n";
      break;
    }

    // Set a timer to be handled at the end of the stream's expected duration
    // (if the stream does not already signal its end using an RTSP "BYE"). This
    // is optional. If, instead, you want to keep the stream active - e.g., so
    // you can later 'seek' back within it and do another RTSP "PLAY" - then you
    // can omit this code. (Alternatively, if you don't want to receive the
    // entire stream, you could set this timer for some shorter value) This
    // ensures that the client handles the stream's end properly

    // Btw, The duration won't be set if it's a live stream
    if (scs.duration > 0) {
      unsigned const delaySlop =
          2; // number of seconds extra to delay, after the stream's expected
             // duration. (This is optional)
      scs.duration += delaySlop;
      unsigned uSecsToDelay =
          (unsigned)(scs.duration * 1000000); // Convert second into microsecond

      // This schedules a task to be executed after the stream's duration has
      // passed. The task scheduled here is stream streamTimerHandler, which is
      // responsible for handling the stream's completion or timeout.
      scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(
          uSecsToDelay, (TaskFunc *)streamTimerHandler, rtspClient);
    }

    env << *rtspClient << "Started playing session ";
    if (scs.duration > 0) {
      env << " (for up to" << scs.duration << " seconds)";
    }
    env << "...\n";

    success = True;
  } while (0);
  delete[] resultString;

  if (!success) {
    shutdownStream(rtspClient);
  }
}

// Implementation of the other event handlers

void subsessionAfterPlaying(void *clientData) {
  // Alias
  MediaSubsession *subsession = (MediaSubsession *)clientData;
  RTSPClient *rtspClient = (RTSPClient *)(subsession->miscPtr);

  // Begin by closing the subsession's stream;
  Medium::close(subsession->sink);
  subsession->sink = NULL;

  // Next, check whether all subsessions' streams have now been closed.
  MediaSession &session = subsession->parentSession();
  MediaSubsessionIterator iter(session);
  while ((subsession = iter.next()) != NULL) {
    // This subsession is still active
    if (subsession->sink != NULL)
      return;
  }

  // Now, all the subsessions' streams have been closed, so shutdown the client
  shutdownStream(rtspClient);
}

void subsessionByeHandler(void *clientData, char const *reason) {
  // Alias
  MediaSubsession *subsession = (MediaSubsession *)clientData;
  RTSPClient *rtspClient = (RTSPClient *)subsession->miscPtr;
  UsageEnvironment &env = rtspClient->envir();

  env << *rtspClient << "Received RTCP \"BYE\"";
  if (reason != NULL) {
    env << " (reason:\"" << reason << "\")";
    delete[] (char *)reason;
  }
  env << " on \"" << *subsession << "\" subsession \n";

  // Now act as if the subsession had closed
  subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void *clientData) {
  // Alias
  ourRTSPClient *rtspClient = (ourRTSPClient *)clientData;
  StreamClientState &scs = rtspClient->scs;

  scs.streamTimerTask = NULL;

  // Shut down the stream;
  shutdownStream(rtspClient);
}

void shutdownStream(RTSPClient *rtspClient, int exitCode) {
  // Alias
  UsageEnvironment &env = rtspClient->envir();
  StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs;

  // First, check whether any subsessions have still to be closed;
  if (scs.session != NULL) {
    Boolean someSubsessionsWereActive = False;
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession *subsession;

    while ((subsession = iter.next()) != NULL) {
      if (subsession->sink != NULL) {
        Medium::close(subsession->sink);
        subsession->sink = NULL;

        // In case the server sends a RTCP "BYE" while handling "TEARDOWN"
        if (subsession->rtcpInstance() != NULL) {
          subsession->rtcpInstance()->setByeHandler(NULL, NULL);
        }
        someSubsessionsWereActive = True;
      }
    }

    if (someSubsessionsWereActive) {
      // Send an RTSP "TEARDOWN" command to tell the server to shutdown the
      // stream. Don't bother handling the response to the "TEARDOWN".
      rtspClient->sendTeardownCommand(*scs.session, NULL);
    }
  }

  env << *rtspClient << "Closing the stream. \n";
  Medium::close(rtspClient);

  if (--rtspClientCount == 0) {

    exit(exitCode);
  }
}

// Implementation of "ourRTSPClient":

ourRTSPClient *ourRTSPClient::createNew(UsageEnvironment &env,
                                        char const *rtspURL, int verbosityLevel,
                                        char const *applicationName,
                                        portNumBits tunnelOverHTTPPortNum) {
  return new ourRTSPClient(env, rtspURL, verbosityLevel, applicationName,
                           tunnelOverHTTPPortNum);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment &env, char const *rtspURL,
                             int verbosityLevel, char const *applicationName,
                             portNumBits tunnelOverHTTPPortNum)
    : RTSPClient(env, rtspURL, verbosityLevel, applicationName,
                 tunnelOverHTTPPortNum, -1) {}

ourRTSPClient::~ourRTSPClient() {
  is_run1 = False;
  is_run2 = False;
  is_run3 = False;
  cv1.notify_all();
  cv2.notify_all();
  cv3.notify_all();
}

// Implementation of "StreamClientState" :

StreamClientState::StreamClientState()
    : iter(NULL), session(NULL), subsession(NULL), streamTimerTask(NULL),
      duration(0.0) {}

StreamClientState::~StreamClientState() {
  delete iter;
  if (session != NULL) {
    // We also need to delete "session", and unschedule "steamTimerTask" (if
    // set) Alias
    UsageEnvironment &env = session->envir();

    env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
    Medium::close(session);
  }
}

// Implementation of "DummySink"

// Even though we're not going to be doing anything with the incoming data, we
// still need to receive it. Define the size of the buffer that we'll use:
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 10000000

DummySink *DummySink::createNew(UsageEnvironment &env,
                                MediaSubsession &subsession,
                                char const *streamId) {
  return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment &env, MediaSubsession &subsession,
                     char const *streamId)
    : MediaSink(env), fSubsession(subsession) {
  fStreamId = strDup(streamId);
  fReceiveBuffer = new u_int8_t[DUMMY_SINK_RECEIVE_BUFFER_SIZE];

  std::string fileName = std::string("./save/")+ DATE1 + "/output_" + oss.str() + ".h264";
  fOutputFile = fopen(fileName.c_str(), "wb");
  std::string accfileName  = std::string("./save/")+DATE1 +"/audio.aac";
  aacFile     = fopen(accfileName.c_str(), "wb");
}

DummySink::~DummySink() {
  fclose(fOutputFile);
  fclose(aacFile);
  delete[] fReceiveBuffer;
  delete[] fStreamId;
}

void DummySink::afterGettingFrame(void *clientData, unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned durationInMicroseconds) {
  DummySink *sink = (DummySink *)clientData;
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime,
                          durationInMicroseconds);
}

// If you don't wanna see debugging output for each received frame, then comment
// out the following line:
#define DEBUG_PRINT_EACH_RECEIVED_FRAME 1

void DummySink::afterGettingFrame(unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned durationInMicroseconds) {
  // We've just received a frame of data. (Optionally) print out information
  // about it:
#ifdef DEBUG_PRINT_EACH_RECEIVED_FRAME
  if (fStreamId != NULL)
    envir() << "Stream \"" << fStreamId << "\";";
  envir() << fSubsession.mediumName() << "/" << fSubsession.codecName()
          << ":\tReceived " << frameSize << " bytes";
  if (numTruncatedBytes > 0)
    envir() << " (with " << numTruncatedBytes << "bytes truncated";
  char uSecsStr[6 + 1];
  snprintf(uSecsStr, sizeof(uSecsStr), "%06u",
           (unsigned)presentationTime.tv_usec);
  envir() << ".\tPresentation time: " << (int)presentationTime.tv_sec << "."
          << uSecsStr;

  if (fSubsession.rtpSource() != NULL &&
      !fSubsession.rtpSource()->hasBeenSynchronizedUsingRTCP()) {
    envir() << "!";
  }

#ifdef DEBUG_PRINT_NPT
  envir() << "\tNPT: " << fSubsession.getNormalPlayTime(presentationTime);
#endif
  envir() << "\n";
#endif
if (fSubsession.mediumName() == std::string("video")){

  if(!inc_head(fReceiveBuffer, 0))
  {
    const uint8_t startCode[] = {0x00, 0x00, 0x00, 0x01};
    count264++;
    if (count264 <= maxFrame)
    {
      if (fSubsession.codecName() == std::string("H264")) {
      // Add NAL start code if it's missing   
      fwrite(startCode, 1, sizeof(startCode), fOutputFile);
      }
      fwrite(fReceiveBuffer, 1, frameSize, fOutputFile);
    }

    uint8_t *copyData = new uint8_t[frameSize + 4];
    memmove(copyData, startCode, 4);
    memmove(copyData + 4, fReceiveBuffer, frameSize);
    std::unique_lock<std::mutex> lock1(mutex1);
    queue264.push(std::make_pair(copyData, frameSize + 4));
    cv1.notify_one();
  }

  else
  {
    count264++;
    if (count264 <= maxFrame)
    {
      fwrite(fReceiveBuffer, 1, frameSize, fOutputFile);
    }

    uint8_t *copyData = new uint8_t[frameSize];
    memmove(copyData, fReceiveBuffer, frameSize);
    std::unique_lock<std::mutex> lock1(mutex1);
    queue264.push(std::make_pair(copyData, frameSize));
    cv1.notify_one();
  }

}

else if(fSubsession.mediumName() == std::string("audio")){

  if(ad.is_set == true)
  {
    if(!inc_head(fReceiveBuffer, 1))
    {
      uint8_t adtsHeader[7];
      addADTSHeader(adtsHeader, frameSize, ad.profile, ad.sampleRate, ad.channels);
      // Write the ADTS header followed by the AAC frame data
      fwrite(adtsHeader, 1, sizeof(adtsHeader), aacFile);
      fwrite(fReceiveBuffer, 1, frameSize, aacFile);
    }
  }
  if(inc_head(fReceiveBuffer, 1))
    // Write the ADTS header followed by the AAC frame data
    fwrite(fReceiveBuffer, 1, frameSize, aacFile);
}
  continuePlaying();
}

Boolean DummySink::continuePlaying() {
  if (fSource == NULL)
    return False;

  fSource->getNextFrame(fReceiveBuffer, DUMMY_SINK_RECEIVE_BUFFER_SIZE,
                        afterGettingFrame, this, onSourceClosure, this);

  return True;
}

static void thread_decode() {

  // Initialize the decode environment
  int id = 0;
  int result;
  set = True;
  const AVCodec *codec;
#ifdef _WIN32
  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
#elif defined(__APPLE__)
  // codec = avcodec_find_decoder_by_name("h264_videotoolbox");
  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
#endif
  if (!codec) {
    std::cerr << "Failed to find the decoder" << std::endl;
    return;
  }

  AVCodecContext *ctx = avcodec_alloc_context3(codec);
  if (!ctx) {
    std::cerr << "Failed to allocate video codec context" << std::endl;
    return;
  }

  if (avcodec_open2(ctx, codec, nullptr) < 0) {
    std::cerr << "Couldn't open the codec!" << std::endl;
    avcodec_free_context(&ctx);
    return;
  }


  AVPacket *pkt = av_packet_alloc();
  uint8_t *data;
  int size;
  while (1) {

    std::unique_lock<std::mutex> lock1(mutex1);
    cv1.wait(lock1, []() { return !queue264.empty(); });
    if (!is_run1)
      break;
    auto nalData = queue264.front();
    queue264.pop();
    data = nalData.first;
    size = nalData.second;

    pkt->data = data;
    pkt->size = size;

    result = avcodec_send_packet(ctx, pkt);

    if (result < 0) {
      std::cerr << "Error sending packet for decoding, result = " << result << std::endl;
      continue;
    }

    while (1) {

      AVFrame *frame = av_frame_alloc();
      result = avcodec_receive_frame(ctx, frame);
      if (result == AVERROR(EAGAIN) || result == AVERROR_EOF || result < 0) {
        av_frame_free(&frame);
        break;
      }

      if (set) {
        std::unique_lock<std::mutex> lockS(m_set);
        fd.height = frame->height;
        fd.width = frame->width;
        fd.format = static_cast<AVPixelFormat>(frame->format);
        set = False;
        cv_set.notify_one();
      }
      id++;
      std::cout << "Decode frame:" << id << " Width: " << frame->width
                << " Height: " << frame->height << std::endl;

      std::unique_lock<std::mutex> lock2(mutex2);
      queueFrame.push(frame);
      cv2.notify_one();
    }
    delete[] data;
    data = nullptr;
  }
  av_packet_free(&pkt);
  avcodec_free_context(&ctx);
}

static void thread_codec() {

  int result;
  const AVCodec *codec;
  initialize_python_environment();
#ifdef _WIN32
  codec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
#elif defined(__APPLE__)
  // codec = avcodec_find_encoder_by_name("hevc_videotoolbox");
  codec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
#endif
  if (!codec) {
    std::cerr << "Failed to find the encoder!" << std::endl;
    return;
  }

  AVCodecContext *ctx = avcodec_alloc_context3(codec);
  if (!ctx) {
    std::cerr << "Failed to allocate video codec context!" << std::endl;
    return;
  }

  std::unique_lock<std::mutex> lockS(m_set);
  cv_set.wait(lockS, []() { return !set; });

  ctx->width   = fd.width;
  ctx->height  = fd.height;
  ctx->pix_fmt = fd.format;

  ctx->time_base.num = 1;
  ctx->time_base.den = 25;

  if (avcodec_open2(ctx, codec, NULL) < 0) {
    std::cerr << "Couldn't open codec" << std::endl;
    return;
  }

  AVFrame *frame;
  AVFrame *res;
  while (1) {

    std::unique_lock<std::mutex> lock2(mutex2);
    cv2.wait(lock2, []() { return !queueFrame.empty(); });
    if (!is_run2)
      break;
    frame = queueFrame.front();
    queueFrame.pop();

    py_process(frame, res);

    if (avcodec_send_frame(ctx, res) < 0) {
      std::cerr << "Error sending a frame for encoding" << std::endl;
      continue;
    }

    while (1) {
      AVPacket *pkt = av_packet_alloc();
      result = avcodec_receive_packet(ctx, pkt);
      if (result == AVERROR(EAGAIN) || result == AVERROR_EOF || result < 0) {
        av_packet_free(&pkt);
        break;
      }
      uint8_t *data = new uint8_t[pkt->size];
      memmove(data, pkt->data, pkt->size);
      int size = pkt->size;
      av_packet_free(&pkt);
      std::unique_lock<std::mutex> lock3(mutex3);
      queue265.push(std::make_pair(data, size));
      cv3.notify_one();
    }
    av_frame_free(&frame);
    av_frame_free(&res);
  }
  avcodec_free_context(&ctx);
}

static void thread_save() {

  int count = 0;
  FILE *h265;
  std::string fileName = std::string("./save/")+ DATE1 + "/output_" + oss.str() + ".h265";
  h265 = fopen(fileName.c_str(), "wb");

  while (1) {
    std::unique_lock<std::mutex> lock3(mutex3);
    cv3.wait(lock3, []() { return !queue265.empty(); });
    if (!is_run3)
      break;
    auto data265 = queue265.front();
    queue265.pop();
    uint8_t *data = data265.first;
    int size = data265.second;
    count++;
    fwrite(data, 1, size, h265);
    if (count == maxFrame)
      exit(0);
    delete[] data;
  }
  fclose(h265);
}

static void addADTSHeader(uint8_t* adtsHeader, int dataLength, int profile, int sampleRate, int channels) {
    int adtsLength = dataLength + 7;  // ADTS header is 7 bytes

    // Syncword 12 bits: always 0xFFF
    adtsHeader[0] = 0xFF;
    adtsHeader[1] = 0xF1; // Syncword and MPEG version (2 bits)

    // MPEG-4, Layer (always 0), protection absent (no CRC)
    adtsHeader[2] = ((profile - 1) << 6) | (sampleRate << 2) | (channels >> 2);
    
    // Channels and length (13 bits)
    adtsHeader[3] = ((channels & 3) << 6) | ((adtsLength >> 11) & 0x3);
    adtsHeader[4] = (adtsLength >> 3) & 0xFF;
    adtsHeader[5] = ((adtsLength & 7) << 5) | 0x1F; // Buffer fullness (0x7FF for variable bitrate)
    adtsHeader[6] = 0xFC;
}

static void ad_set(char* descripition)
{
    std::string sdpContent(descripition);
      // Regular expression to search for "config" and capture its value
    std::regex configRegex(R"(config=(\w+))");
    std::smatch match;
    std::string configValue;

    // Search for config value in the SDP content
    if (std::regex_search(sdpContent, match, configRegex)) {
        configValue = match[1].str();  // Capturing the value
        std::cout << "Found config value: " << configValue << std::endl;
    } else {
        std::cout << "Config not found" << std::endl;
        return;
    }
    //short convertedValue = static_cast<short>(std::stoi(configValue));
    uint8_t a1 = static_cast<uint8_t>(configValue[0]);
    uint8_t a2 = static_cast<uint8_t>(configValue[1]);
    uint8_t a3 = static_cast<uint8_t>(configValue[2]);
    uint8_t a4 = static_cast<uint8_t>(configValue[3]);

    unsigned short convertedValue = 0;
    convertedValue |= (a1 & 0x0F) << 12;  // a1 (first nibble) -> bits 15-12
    convertedValue |= (a2 & 0x0F) << 8;   // a2 (second nibble) -> bits 11-8
    convertedValue |= (a3 & 0x0F) << 4;   // a3 (third nibble) -> bits 7-4
    convertedValue |= (a4 & 0x0F); 

    ad.profile    = (convertedValue >> 11)   & 0x1F;
    ad.sampleRate = (convertedValue >> 7) & 0x0F;
    ad.channels   = (convertedValue >> 3)   & 0x0F;
    ad.is_set     = true;
}

static bool inc_head(uint8_t* data, int type){
  if(type == 1){
    if((data[0] == 0xFF) && ((data[1]&0xF0)==0xF0))
      return true;
    else 
      return false;
  }

  else if(type == 0){
    if((data[0] == 0x00) && (data[1] == 0x00) && (data[2] == 0x00) && ((data[3] == 0x01)||(data[3] == 0x00)))
      return true;
    else
      return false;
  }
}