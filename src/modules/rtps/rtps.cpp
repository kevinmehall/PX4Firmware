/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rtps.c
 * RTPS to uORB bridge
 *
 * @author Kevin Mehall <kevin.mehall@3drobotics.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/optical_flow.h>
#include <redrider.capnp.h>

#include <commkit/node.h>
#include <commkit/subscriber.h>

static volatile bool keep_running;
static volatile bool started = false;

struct ORB_ToRTPS_Base {
  commkit::PublisherPtr publisher;
  const orb_metadata* orb_id;
  int orb_fd;
  
  virtual void poll_matched() {}
  
  virtual ~ORB_ToRTPS_Base() {}
};

template<typename CapnType, typename orb_type>
struct ORB_ToRTPS: ORB_ToRTPS_Base {
  ORB_ToRTPS(commkit::Node& node, const char* rtps_topic, bool reliable, const orb_metadata* orb) {
    auto topic = commkit::Topic::capn<CapnType>(rtps_topic);
    commkit::PublicationOpts pub_opts;
    pub_opts.reliable = reliable;
    pub_opts.history = 0;
    
    publisher = node.createPublisher(topic);
    if (publisher == nullptr) {
        warnx("Error creating publisher for %s\n", rtps_topic);
        return;
    }
    if (!publisher->init(pub_opts)) {
        warnx("Error initializing publisher for %s\n", rtps_topic);
        return;
    }
    
    orb_id = orb;
    orb_fd = orb_subscribe(orb);
  }
  
  void poll_matched() {
    orb_type data;
    orb_copy(orb_id, orb_fd, &data);
    on_orb(data);
  }
  
  virtual void on_orb(orb_type&) {}
};

struct ORB_FromRTPS_Base {
  commkit::SubscriberPtr subscriber;
  const orb_metadata* orb_id;
  
  virtual ~ORB_FromRTPS_Base() {}
};

template<typename CapnType, typename orb_type>
struct ORB_FromRTPS: ORB_FromRTPS_Base {
  ORB_FromRTPS(commkit::Node& node, const char* rtps_topic, bool reliable, const orb_metadata* orb) {
    auto topic = commkit::Topic::capn<CapnType>(rtps_topic);
    commkit::SubscriptionOpts sub_opts;
    sub_opts.reliable = reliable;
    sub_opts.history = 0;
    
    subscriber = node.createSubscriber(topic);
    if (subscriber == nullptr) {
        warnx("Error creating subscriber for %s\n", rtps_topic);
        return;
    }
    subscriber->onMessage.connect(&ORB_FromRTPS::rtps_recv, this);
    if (!subscriber->init(sub_opts)) {
        warnx("Error initializing subscriber for %s\n", rtps_topic);
        return;
    }
    
    orb_id = orb;
  }
  
  virtual void on_rtps(typename CapnType::Reader) {}
  
  void rtps_recv(commkit::SubscriberPtr sub) {
    commkit::Payload payload;
    while (sub->take(&payload)) {
      auto capn = payload.toReader<CapnType>();
      on_rtps(capn);
    }
  }
};

struct OpticalFlow_FromRTPS: ORB_FromRTPS<redrider::OpticalFlow, optical_flow_s> {
  OpticalFlow_FromRTPS(commkit::Node& node): ORB_FromRTPS(node, "OpticalFlow", false, ORB_ID(optical_flow)) {}
  
  void on_rtps(redrider::OpticalFlow::Reader reader) {
    printf("optflow: %f\n", (double) reader.getFlow().getX());
  }
};

struct Gyro_ToRTPS: ORB_ToRTPS<redrider::Gyro, sensor_gyro_s> {
  Gyro_ToRTPS(commkit::Node& node): ORB_ToRTPS(node, "Gyro", false, ORB_ID(sensor_gyro)) {}
  
  void on_orb(sensor_gyro_s& gyro) {
    capnp::MallocMessageBuilder mb;
    auto p_gyro = mb.getRoot<redrider::Gyro>();
    p_gyro.setTimestamp(gyro.timestamp);
    p_gyro.getIntegral().setX(gyro.x_integral);
    p_gyro.getIntegral().setY(gyro.y_integral);
    p_gyro.getIntegral().setZ(gyro.x_integral);
    p_gyro.setIntegralDt(gyro.integral_dt);
    publisher->publish(mb);
  }
};

static int thread_main() {
  commkit::Node node;
  if (!node.init("px4")) {
      warnx("Error initializing commkit\n");
      return 1;
  }
  
  std::unique_ptr<ORB_ToRTPS_Base> to_rtps[] = {
    std::unique_ptr<ORB_ToRTPS_Base>(new Gyro_ToRTPS(node)),
  };
  
  std::unique_ptr<ORB_FromRTPS_Base> from_rtps[] = {
    std::unique_ptr<ORB_FromRTPS_Base>(new OpticalFlow_FromRTPS(node)),
  };
  
  const size_t to_rtps_count = sizeof(to_rtps) / sizeof(to_rtps[0]);
  px4_pollfd_struct_t fds[to_rtps_count];
  
  for (size_t i = 0; i<to_rtps_count; i++) {
    fds[i] = { .fd = to_rtps[i]->orb_fd, .events = POLLIN };
  }
  
  while (keep_running) {
    px4_poll(fds, 1, 1000);
    for (size_t i = 0; i<to_rtps_count; i++) {
      if (fds[i].revents & POLLIN) {
        to_rtps[i]->poll_matched();
      }
    }
  }
  
  (void) from_rtps;

  started = false;
  return OK;
}

static int start() {
  if (started) {
    warnx("Already running");
    return 1;
  }

  started = true;
  keep_running = true;
  px4_task_spawn_cmd("rtps",
         SCHED_DEFAULT,
         SCHED_PRIORITY_DEFAULT,
         2700,
         (px4_main_t)&thread_main,
         0);

  return OK;
}

static int stop() {
  keep_running = false;
  while (started) usleep(10000);
  return OK;
}

extern "C" __EXPORT int rtps_main(int argc, char *argv[]);

extern "C" int rtps_main(int argc, char *argv[])
{
    if (!strcmp(argv[1], "start")) {
      return start();

    } else if (!strcmp(argv[1], "stop")) {
      return stop();

    } else {
      	warnx("usage: rtps {start|stop}");
        return 1;
    }

    return OK;
}
