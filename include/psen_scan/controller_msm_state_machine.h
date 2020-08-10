// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_CONTROLLER_MSM_STATE_MACHINE_H
#define PSEN_SCAN_CONTROLLER_MSM_STATE_MACHINE_H

#include <iostream>
#include <functional>

// back-end
#include <boost/msm/back/state_machine.hpp>
// front-end
#include <boost/msm/front/state_machine_def.hpp>

namespace psen_scan
{

namespace msm = boost::msm;
namespace mpl = boost::mpl;

// events
struct start_request_event
{
};
struct start_reply_received_event
{
};
struct monitoring_frame_received_event
{
};
struct stop_request_event
{
};
struct stop_reply_received_event
{
};

using SendStartRequestCallback = std::function<void()>;

// front-end: define the FSM structure
struct msm_front_ : public msm::front::state_machine_def<msm_front_>
{

  // The list of FSM states
  struct InitState : public msm::front::state<>
  {
    SendStartRequestCallback send_start_request_callback_;
    // every (optional) entry/exit methods get the event passed.
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
    }

    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      // std::cerr << "leaving: InitState\n\n";
      if (send_start_request_callback_)
      {
        send_start_request_callback_();
      }
    }
  } InitState_;

  struct WaitForStartReplyState : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
    }
  };
  struct WaitForMonitoringFrameState : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
    }
  };
  struct WaitForStopReplyState : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
    }
  };

  typedef InitState initial_state;

  // TODO: m is not used? Necessary?
  typedef msm_front_ m;  // makes transition table cleaner

  // TODO: Use Action for function call?

  // Transition table for the scanner
  // clang-format off
  struct transition_table : mpl::vector<
      //    Start                         Event                            Next           			 Action	     Guard
      //  +-------------------------------+--------------------------------+---------------------------+---------+---+
      _row < InitState,                   start_request_event,             WaitForStartReplyState                    >,
      _row < WaitForStartReplyState,      start_reply_received_event,      WaitForMonitoringFrameState               >,
      _row < WaitForMonitoringFrameState, monitoring_frame_received_event, WaitForMonitoringFrameState               >,
      _row < WaitForMonitoringFrameState, stop_request_event,              WaitForStopReplyState                     >,
      _row < WaitForStopReplyState,       stop_reply_received_event,       InitState                                 >
      //  +-------------------------------+--------------------------------+---------------------------+---------+---+
  > {};
  // clang-format on
  // Replaces the default no-transition response.
  template <class FSM, class Event>
  void no_transition(Event const& e, FSM&, int state)
  {
      //TODO Implement handling
  }
};

// Pick a back-end
typedef msm::back::state_machine<msm_front_> controller_msm_state_machine;

}  // namespace psen_scan

#endif  // PSEN_SCAN_CONTROLLER_MSM_STATE_MACHINE_H