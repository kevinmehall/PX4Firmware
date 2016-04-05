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
#include <demo.capnp.h>

#include <commkit/node.h>
#include <commkit/subscriber.h>

static volatile bool keep_running;
static volatile bool started = false;

static void onOpticalFlowMessage(commkit::SubscriberPtr sub) {
    commkit::Payload payload;
    while (sub->take(&payload)) {
      auto mytype = payload.toReader<OpticalFlow>();
      printf("optflow: %f\n", (double) mytype.getFlowX());
    }
}

static int thread_main() {
  commkit::Node node;
  if (!node.init("px4")) {
      warnx("Error initializing commkit\n");
      return 1;
  }
  
  commkit::SubscriptionOpts sub_opts;
  sub_opts.reliable = false;
  sub_opts.history = false;
  
  commkit::PublicationOpts pub_opts;
  pub_opts.reliable = false;
  pub_opts.history = false;

  commkit::Topic optical_flow_topic("OpticalFlow", "capnp+OpticalFlow", 512);
  auto optical_flow_sub = node.createSubscriber(optical_flow_topic);
  if (optical_flow_sub == nullptr) {
      warnx("error\n");
      return 1;
  }
  optical_flow_sub->onMessage.connect(&onOpticalFlowMessage);
  if (!optical_flow_sub->init(sub_opts)) {
      warnx("error\n");
      return 1;
  }
  
  commkit::Topic gyro_topic("Gyro", "capnp+Gyro", 512);
  auto gyro_pub = node.createPublisher(gyro_topic);
  if (gyro_pub == nullptr) {
      printf("error\n");
      return 1;
  }
  if (!gyro_pub->init(pub_opts)) {
      printf("error\n");
      exit(1);
  }
  

  int gyro_sub_fd = orb_subscribe(ORB_ID(sensor_gyro));
  /* one could wait for multiple topics with this technique, just using one here */
  px4_pollfd_struct_t fds[] = {
      { .fd = gyro_sub_fd,   .events = POLLIN },
  };

  while (keep_running) {
    px4_poll(fds, 1, 1000);
    if (fds[0].revents & POLLIN) {
        struct sensor_gyro_s gyro;
        orb_copy(ORB_ID(sensor_gyro), gyro_sub_fd, &gyro);
        
        capnp::MallocMessageBuilder mb;
        auto p_gyro = mb.getRoot<Gyro>();
        p_gyro.setTimestamp(gyro.timestamp);
        p_gyro.setXIntegral(gyro.x_integral);
        p_gyro.setYIntegral(gyro.y_integral);
        p_gyro.setZIntegral(gyro.x_integral);
        p_gyro.setIntegralDt(gyro.integral_dt);
        gyro_pub->publish(mb);
    }
  }

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
