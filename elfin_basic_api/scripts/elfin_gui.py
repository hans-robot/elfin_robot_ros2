#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 28 12:18:05 2017

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2017, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 
"""
# author: Cong Liu

from __future__ import division
from tokenize import Double
import rclpy
from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
from rcl_interfaces.msg import SetParametersResult
import rclpy.parameter
from rclpy.node import Node
from rclpy.duration import Duration
import time
from rclpy.timer import Timer
from rclpy.action import ActionClient
import math
import os # 20201209: add os path
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from std_msgs.msg import Bool, String,Float32
from  std_srvs.srv._set_bool import SetBool,SetBool_Request, SetBool_Response
from elfin_robot_msgs.srv._elfin_iod_read import ElfinIODRead_Request, ElfinIODRead_Response,ElfinIODRead
from elfin_robot_msgs.srv._elfin_iod_write import ElfinIODWrite_Request, ElfinIODWrite_Response,ElfinIODWrite
from elfin_robot_msgs.srv._set_int16 import SetInt16_Request, SetInt16_Response,SetInt16
from elfin_robot_msgs.srv._set_string import SetString_Request,SetString_Response,SetString
import moveit_msgs.action._move_group
import wx
from sensor_msgs.msg import JointState
import transforms3d
import geometry_msgs.msg
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Quaternion
from control_msgs.action import FollowJointTrajectory
from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_Goal
import threading

class MyFrame(wx.Frame,Node):  
  
    def __init__(self,parent,id):  
        rclpy.init(args=None)
        self.node = rclpy.create_node('elfin_basic_gui')
        self.gui_node = rclpy.create_node('elfin_gui_node_test')
 
        the_size=(700, 720) # height from 550 change to 700
        wx.Frame.__init__(self,parent,id,'Elfin Control Panel',pos=(250,100)) 
        self.panel=wx.Panel(self)
        font=self.panel.GetFont()
        font.SetPixelSize((12, 24))
        self.panel.SetFont(font)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,self.gui_node)

        self.callback_group = ReentrantCallbackGroup()
        
        self.controller_ns="elfin_arm_controller/"
        self.elfin_driver_ns='elfin_ros_control/elfin/'
        self.elfin_IO_ns='elfin_ros_control/elfin/io_port1/' # 20201126: add IO ns

        self.publisher_ = self.node.create_publisher(Float32, 'vel', 10)

        self.call_read_do_req = ElfinIODRead_Request() # ElfinIODReadRequest()
        self.call_read_di_req = ElfinIODRead_Request() # ElfinIODReadRequest()
        self.call_read_do_req.data = True
        self.call_read_di_req.data = True
        self.call_read_do = self.node.create_client(ElfinIODRead, '/read_do')
        self.call_read_di = self.node.create_client(ElfinIODRead, '/read_di')

        # 20201126: add service for write_do
        self.call_write_DO=self.node.create_client(ElfinIODWrite, '/write_do')
        
        self.elfin_basic_api_ns='elfin_basic_api/'

        self.node.declare_parameter('use_fake_robot', True)
        self.use_fake_robot = self.node.get_parameter('use_fake_robot').get_parameter_value().bool_value

        self.node.declare_parameter(self.controller_ns+"joints", ["elfin_joint1","elfin_joint2","elfin_joint3","elfin_joint4","elfin_joint5","elfin_joint6"])
        self.joint_names=self.node.get_parameter(self.controller_ns+"joints").get_parameter_value().string_array_value
        
        self.ref_link_name="world" # self.group.get_planning_frame()
        self.end_link_name="elfin_end_link" # self.group.get_end_effector_link()
        
        self.ref_link_lock=threading.Lock()
        self.end_link_lock=threading.Lock()
        self.DO_btn_lock = threading.Lock() # 20201208: add the threading lock
        self.DI_show_lock = threading.Lock()

        self.node_lock = threading.Lock()

        self.current_joint_val=[0]*6
        self.js_display=[0]*6 # joint_states
        self.jm_button=[0]*6 # joints_minus
        self.jp_button=[0]*6 # joints_plus
        self.js_label=[0]*6 # joint_states
                      
        self.ps_display=[0]*6 # pcs_states
        self.pm_button=[0]*6 # pcs_minus
        self.pp_button=[0]*6 # pcs_plus
        self.ps_label=[0]*6 # pcs_states

        # 20201208: add the button array
        self.DO_btn_display=[0]*4 # DO states
        self.DI_display=[0]*4 # DI states
        self.LED_display=[0]*4 # LED states
        self.End_btn_display=[0]*4 # end button states

        self.btn_height=370 # 20201126: from 390 change to 370
        self.btn_path = os.path.dirname(os.path.realpath(__file__)) # 20201209: get the elfin_gui.py path
        btn_lengths=[]
        self.DO_DI_btn_length=[0,92,157,133] # 20201209: the length come from servo on, servo off, home, stop button
        self.btn_interstice=22 # 20201209: come from btn_interstice

        self.display_init()              
        self.key=[]
        self.DO_btn=[0,0,0,0,0,0,0,0] # DO state, first four bits is DO, the other is LED
        self.DI_show=[0,0,0,0,0,0,0,0] # DI state, first four bits is DI, the other is the end button
                
        self.power_on_btn=wx.Button(self.panel, label=' Servo On ', name='Servo On',
                                    pos=(20, self.btn_height))
        btn_lengths.append(self.power_on_btn.GetSize()[0])
        btn_total_length=btn_lengths[0]
        
        self.power_off_btn=wx.Button(self.panel, label=' Servo Off ', name='Servo Off')
        btn_lengths.append(self.power_off_btn.GetSize()[0])
        btn_total_length+=btn_lengths[1]
        
        self.reset_btn=wx.Button(self.panel, label=' Clear Fault ', name='Clear Fault')
        btn_lengths.append(self.reset_btn.GetSize()[0])
        btn_total_length+=btn_lengths[2]

        self.home_btn=wx.Button(self.panel, label='Home', name='home_btn')
        btn_lengths.append(self.home_btn.GetSize()[0])
        btn_total_length+=btn_lengths[3]
        
        self.stop_btn=wx.Button(self.panel, label='Stop', name='Stop')
        btn_lengths.append(self.stop_btn.GetSize()[0])
        btn_total_length+=btn_lengths[4]

        self.btn_interstice=(550-btn_total_length)/4
        btn_pos_tmp=btn_lengths[0]+self.btn_interstice+20 # 20201126: 20:init length + btn0 length + btn_inter:gap
        self.power_off_btn.SetPosition((int(btn_pos_tmp), self.btn_height))
        
        btn_pos_tmp+=btn_lengths[1]+self.btn_interstice
        self.reset_btn.SetPosition((int(btn_pos_tmp), self.btn_height))
        
        btn_pos_tmp+=btn_lengths[2]+self.btn_interstice
        self.home_btn.SetPosition((int(btn_pos_tmp), self.btn_height))
        
        btn_pos_tmp+=btn_lengths[3]+self.btn_interstice
        self.stop_btn.SetPosition((int(btn_pos_tmp), self.btn_height))
        
        self.servo_state_label=wx.StaticText(self.panel, label='Servo state:',
                                              pos=(590, self.btn_height-10))
        self.servo_state_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', pos=(600, self.btn_height+10))
        self.servo_state=bool()
        
        self.servo_state_lock=threading.Lock()
        
        self.fault_state_label=wx.StaticText(self.panel, label='Fault state:',
                                              pos=(590, self.btn_height+60))
        self.fault_state_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', pos=(600, self.btn_height+80))
        self.fault_state=bool()
        
        self.fault_state_lock=threading.Lock()

        # 20201209: add the description of end button
        self.end_button_state_label=wx.StaticText(self.panel, label='END Button state',
                                            pos=(555,self.btn_height+172))
        
        self.reply_show_label=wx.StaticText(self.panel, label='Result:',
                                           pos=(20, self.btn_height+260)) # 20201126: btn_height from 120 change to 260.
        self.reply_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', size=(670, 30), pos=(20, self.btn_height+280))# 20201126: btn_height from 140 change to 280.
        
        link_textctrl_length=int((btn_pos_tmp-40)/2)
        
        self.ref_links_show_label=wx.StaticText(self.panel, label='Ref. link:',
                                                    pos=(20, self.btn_height+210)) # 20201126: btn_height from 60 change to 210.
        
        self.ref_link_show=wx.TextCtrl(self.panel, style=(wx.TE_READONLY),
                                           value=self.ref_link_name, size=(link_textctrl_length, 30),
                                           pos=(20, self.btn_height+230)) # 20201126: btn_height from 80 change to 230.
        
        self.end_link_show_label=wx.StaticText(self.panel, label='End link:',
                                               pos=(link_textctrl_length+30, self.btn_height+210))# 20201126: btn_height from 80 change to 200.
        
        self.end_link_show=wx.TextCtrl(self.panel, style=(wx.TE_READONLY),
                                       value=self.end_link_name, size=(link_textctrl_length, 30),
                                       pos=(link_textctrl_length+30, self.btn_height+230))
        
        self.set_links_btn=wx.Button(self.panel, label='Set links', name='Set links')
        self.set_links_btn.SetPosition((int(btn_pos_tmp), self.btn_height+230)) # 20201126: btn_height from 75 change to 220.
        
        # the variables about velocity scaling
        self.node.declare_parameter('velocity_scaling', 0.4)
        velocity_scaling_init=self.node.get_parameter('velocity_scaling').get_parameter_value().double_value
        default_velocity_scaling=str(round(velocity_scaling_init*100, 2))
        self.velocity_setting_label=wx.StaticText(self.panel, label='Velocity Scaling',
                                                  pos=(20, self.btn_height-55)) # 20201126: btn_height from 70 change to 55
        self.velocity_setting=wx.Slider(self.panel, value=int(velocity_scaling_init*100),
                                        minValue=1, maxValue=100,
                                        style = wx.SL_HORIZONTAL,
                                        size=(500, 30),
                                        pos=(45, self.btn_height-35)) # 20201126: btn_height from 70 change to 35
        self.velocity_setting_txt_lower=wx.StaticText(self.panel, label='1%',
                                                    pos=(20, self.btn_height-35)) # 20201126: btn_height from 45 change to 35
        self.velocity_setting_txt_upper=wx.StaticText(self.panel, label='100%',
                                                    pos=(550, self.btn_height-35))# 20201126: btn_height from 45 change to 35
        self.velocity_setting_show=wx.TextCtrl(self.panel, 
                                               style=(wx.TE_CENTER|wx.TE_READONLY), 
                                                value=default_velocity_scaling+'%',
                                                pos=(600, self.btn_height-45))# 20201126: btn_height from 55 change to 45
        self.velocity_setting.Bind(wx.EVT_SLIDER, self.velocity_setting_cb)
 
        self.teleop_api_dynamic_reconfig_client = self.node.add_on_set_parameters_callback(self.basic_api_reconfigure_cb)
        self.dlg=wx.Dialog(self.panel, title='message')
        self.dlg.Bind(wx.EVT_CLOSE, self.closewindow)
        self.dlg_panel=wx.Panel(self.dlg)
        self.dlg_label=wx.StaticText(self.dlg_panel, label='hello', pos=(15, 15))
        
        self.set_links_dlg=wx.Dialog(self.panel, title='Set links', size=(400, 100))
        self.set_links_dlg_panel=wx.Panel(self.set_links_dlg)
        
        self.sld_ref_link_show=wx.TextCtrl(self.set_links_dlg_panel, style=wx.TE_PROCESS_ENTER,
                                           value='', pos=(20, 20), size=(link_textctrl_length, 30))
        self.sld_end_link_show=wx.TextCtrl(self.set_links_dlg_panel, style=wx.TE_PROCESS_ENTER,
                                           value='', pos=(20, 70), size=(link_textctrl_length, 30))
        
        self.sld_set_ref_link_btn=wx.Button(self.set_links_dlg_panel, label='Update ref. link',
                                            name='Update ref. link')
        self.sld_set_ref_link_btn.SetPosition((link_textctrl_length+30, 15))
        self.sld_set_end_link_btn=wx.Button(self.set_links_dlg_panel, label='Update end link',
                                            name='Update end link')
        self.sld_set_end_link_btn.SetPosition((link_textctrl_length+30, 65))
        
        self.set_links_dlg.SetSize((link_textctrl_length+self.sld_set_ref_link_btn.GetSize()[0]+50, 120))
        
                        
        self.call_teleop_joint=self.node.create_client(SetInt16,'/joint_teleop')
        self.call_teleop_joint_req=SetInt16_Request()
        
        self.call_teleop_cart=self.node.create_client(SetInt16, '/cart_teleop')
        self.call_teleop_cart_req=SetInt16_Request()
        
        self.call_teleop_stop=self.node.create_client(SetBool,'/stop_teleop')
        self.call_teleop_stop_req=SetBool_Request()
        
        self.call_stop=self.node.create_client(SetBool,'/stop_teleop')
        self.call_stop_req=SetBool_Request()
        self.call_stop_req.data=True
        self.stop_btn.Bind(wx.EVT_BUTTON, 
                           lambda evt, cl=self.call_stop,
                           rq=self.call_stop_req :
                           self.call_set_bool_common(evt, cl, rq))
            
        self.call_reset=self.node.create_client(SetBool,'/clear_fault')
        self.call_reset_req=SetBool_Request()
        self.call_reset_req.data=True
        self.reset_btn.Bind(wx.EVT_BUTTON, 
                           lambda evt, cl=self.call_reset,
                           rq=self.call_reset_req :
                           self.call_set_bool_common(evt, cl, rq))
        
        if self.use_fake_robot:
            self.call_power_on=self.node.create_client(SetBool,'/elfin_basic_api/enable_robot')
        else:
            self.call_power_on = self.node.create_client(SetBool,'/enable_robot')
        self.call_power_on_req=SetBool_Request()
        self.call_power_on_req.data=True
        self.power_on_btn.Bind(wx.EVT_BUTTON, 
                               lambda evt, cl=self.call_power_on,
                               rq=self.call_power_on_req :
                               self.call_set_bool_common(evt, cl, rq))
        
        if self.use_fake_robot:
            self.call_power_off=self.node.create_client(SetBool,'/elfin_basic_api/disable_robot')
        else:
            self.call_power_off=self.node.create_client(SetBool,'/disable_robot')
        self.call_power_off_req=SetBool_Request()
        self.call_power_off_req.data=True
        self.power_off_btn.Bind(wx.EVT_BUTTON, 
                               lambda evt, cl=self.call_power_off,
                               rq=self.call_power_off_req :
                               self.call_set_bool_common(evt, cl, rq))
                
        self.call_move_homing=self.node.create_client(SetBool,'/home_teleop')
        self.call_move_homing_req=SetBool_Request()
        self.call_move_homing_req.data=True
        self.home_btn.Bind(wx.EVT_LEFT_DOWN, 
                           lambda evt, cl=self.call_move_homing,
                           rq=self.call_move_homing_req :
                           self.call_set_bool_common(evt, cl, rq))
        self.home_btn.Bind(wx.EVT_LEFT_UP,
                           lambda evt, mark=100:
                           self.release_button(evt, mark) )
            
        self.call_set_ref_link=self.node.create_client(SetString, '/set_reference_link')
        self.call_set_end_link=self.node.create_client(SetString, '/set_end_link')
        self.set_links_btn.Bind(wx.EVT_BUTTON, self.show_set_links_dialog)
        
        self.sld_set_ref_link_btn.Bind(wx.EVT_BUTTON, self.update_ref_link)
        self.sld_set_end_link_btn.Bind(wx.EVT_BUTTON, self.update_end_link)
        
        self.sld_ref_link_show.Bind(wx.EVT_TEXT_ENTER, self.update_ref_link)
        self.sld_end_link_show.Bind(wx.EVT_TEXT_ENTER, self.update_end_link)
            
        self.action_client=ActionClient(self.node,FollowJointTrajectory,self.controller_ns+'follow_joint_trajectory')
        self.action_goal=FollowJointTrajectory_Goal()
        self.action_goal.trajectory.joint_names=self.joint_names
        
        self.SetMinSize(the_size)
        self.SetMaxSize(the_size)
    
    def display_init(self):
        js_pos=[20, 20]
        js_btn_length=[70, 70, 61, 80]
        js_distances=[10, 20, 10, 26]
        dis_h=50
        for i in range(len(self.js_display)):
            self.jp_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' +', 
                                        pos=(js_pos[0],
                                             js_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp=js_btn_length[0]+js_distances[0]
                                        
            self.jp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.teleop_joints(evt, mark) )
            self.jp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.release_button(evt, mark) )
            
            self.jm_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' -', 
                                        pos=(js_pos[0]+dis_tmp,
                                             js_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp+=js_btn_length[1]+js_distances[1]
                                        
            self.jm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.teleop_joints(evt, mark) )
            self.jm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.release_button(evt, mark) )
            
            pos_js_label=(js_pos[0]+dis_tmp, js_pos[1]+(5-i)*dis_h)
            self.js_label[i]=wx.StaticText(self.panel,
                                           label='J'+str(i+1)+'/deg:',
                                           pos=pos_js_label)
            self.js_label[i].SetPosition((int(pos_js_label[0]), int(pos_js_label[1]+abs(40-self.js_label[i].GetSize()[1])/2)))
            dis_tmp+=js_btn_length[2]+js_distances[2]

            pos_js_display=(js_pos[0]+dis_tmp, js_pos[1]+(5-i)*dis_h)
            self.js_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value=' 0000.00 ', 
                                           pos=pos_js_display)
            self.js_display[i].SetPosition((int(pos_js_display[0]), int(pos_js_display[1]+abs(40-self.js_display[i].GetSize()[1])/2)))
            dis_tmp+=js_btn_length[3]+js_distances[3]

        ps_pos=[js_pos[0]+dis_tmp, 20]
        ps_btn_length=[70, 70, 53, 80]
        ps_distances=[10, 20, 10, 20]
        pcs_btn_label=['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        pcs_label=['X', 'Y', 'Z', 'R', 'P', 'Y']
        unit_label=['/mm:', '/mm:', '/mm:', '/deg:', '/deg:', '/deg:']
        for i in range(len(self.ps_display)):
            self.pp_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' +', 
                                        pos=(ps_pos[0],
                                             ps_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp=ps_btn_length[0]+ps_distances[0]
                                        
            self.pp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.teleop_pcs(evt, mark) )
            self.pp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.release_button(evt, mark) )
            
            self.pm_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' -', 
                                        pos=(ps_pos[0]+dis_tmp,
                                             ps_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp+=ps_btn_length[1]+ps_distances[1]
                                        
            self.pm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.teleop_pcs(evt, mark) )
            self.pm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.release_button(evt, mark) )
            
            pos_ps_label=(ps_pos[0]+dis_tmp, ps_pos[1]+(5-i)*dis_h)
            self.ps_label[i]=wx.StaticText(self.panel, 
                                           label=pcs_label[i]+unit_label[i],
                                           pos=pos_ps_label)
            self.ps_label[i].SetPosition((int(pos_ps_label[0]), int(pos_ps_label[1]+abs(40-self.ps_label[i].GetSize()[1])/2)))
            dis_tmp+=ps_btn_length[2]+ps_distances[2]
            
            pos_ps_display=(ps_pos[0]+dis_tmp, ps_pos[1]+(5-i)*dis_h)
            self.ps_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value='', 
                                           pos=pos_ps_display)
            self.ps_display[i].SetPosition((int(pos_ps_display[0]), int(pos_ps_display[1]+abs(40-self.ps_display[i].GetSize()[1])/2)))
            dis_tmp+=ps_btn_length[3]+ps_distances[3]

        # 20201209: add the DO,LED,DI,end button.
        for i in range(len(self.DO_btn_display)):
            self.DO_btn_display[i]=wx.Button(self.panel,label='DO'+str(i),
                                        pos=(20+(self.DO_DI_btn_length[i]+self.btn_interstice)*i,
                                        self.btn_height+40))
            self.DO_btn_display[i].Bind(wx.EVT_BUTTON,
                                    lambda evt,marker=i,cl=self.call_write_DO : 
                                    self.call_write_DO_command(evt,marker,cl))

            self.DI_display[i]=wx.TextCtrl(self.panel, style=(wx.TE_CENTER | wx.TE_READONLY), value='DI'+str(i),
                                size=(self.DO_btn_display[i].GetSize()), 
                                pos=(20+(self.DO_DI_btn_length[i]+self.btn_interstice)*i,self.btn_height+80))

            self.LED_display[i]=wx.Button(self.panel,label='LED'+str(i),
                                        pos=(20+(self.DO_DI_btn_length[i]+self.btn_interstice)*i,self.btn_height+120))
            self.LED_display[i].Bind(wx.EVT_BUTTON,
                                    lambda evt, marker=4+i, cl=self.call_write_DO : 
                                    self.call_write_DO_command(evt, marker,cl))

            png=wx.Image(self.btn_path+'/btn_icon/End_btn'+str(i)+'_low.png',wx.BITMAP_TYPE_PNG).ConvertToBitmap()
            self.End_btn_display[i]=wx.StaticBitmap(self.panel,-1,png,
                                                pos=(40+(self.DO_DI_btn_length[i]+self.btn_interstice)*i,
                                                self.btn_height+160))

    def velocity_setting_cb(self, event):
        current_velocity_scaling=self.velocity_setting.GetValue()*0.01
        msg = Float32()
        msg.data = current_velocity_scaling
        self.publisher_.publish(msg)
        self.node.get_logger().info('new vel: %s' % str(current_velocity_scaling))
        vel_new_param = rclpy.parameter.Parameter(
            'velocity_scaling',
            rclpy.Parameter.Type.DOUBLE,
            current_velocity_scaling
        )
        vel_new_parameters = [vel_new_param]
        self.basic_api_reconfigure_cb(vel_new_parameters)
        wx.CallAfter(self.update_velocity_scaling_show, current_velocity_scaling)
    
    def joint_state_cb(self, msg):
        joint_name = msg.name
        static_name = "elfin_joint"
        for i in range(0,len(joint_name)):
            current_joint_name = static_name + str(i+1)
            for j in range(0, len(joint_name)):
                if joint_name[j] == current_joint_name:
                    self.current_joint_val[i] = msg.position[j]
       
    def basic_api_reconfigure_cb(self, params):
        for param in params:
            if self.velocity_setting_show.GetValue()!=param.value:
                self.velocity_setting.SetValue(int(param.value*100))
                self.node.get_logger().info('set new vel: %s' % str(param.value*100))
                wx.CallAfter(self.update_velocity_scaling_show, param.value) 
        return SetParametersResult(successful=True)     
    
    def action_stop(self):
        self.call_teleop_stop_req.data=True
        resp=self.call_teleop_stop.call_async(self.call_teleop_stop_req)
    
    def teleop_joints(self,event,mark):
        try:
            self.node_lock.acquire()
            self.call_teleop_joint_req.data=mark
            resp=self.call_teleop_joint.call_async(self.call_teleop_joint_req)
            rclpy.spin_until_future_complete(self.node, resp)
            wx.CallAfter(self.update_reply_show, resp.result())
            event.Skip()
            self.node_lock.release()
        except Exception as e:
            self.node.get_logger().info('teleop joints error')
        
    def teleop_pcs(self,event,mark):
        try: 
            self.node_lock.acquire()
            self.call_teleop_cart_req.data=mark            
            resp=self.call_teleop_cart.call_async(self.call_teleop_cart_req)
            rclpy.spin_until_future_complete(self.node, resp)
            wx.CallAfter(self.update_reply_show, resp.result())
            event.Skip()
            self.node_lock.release()
        except Exception as e:
            self.node.get_logger().info('eleop pcs error')
    
    def release_button(self, event, mark):
        self.node_lock.acquire()
        self.call_teleop_stop_req.data=True
        resp=self.call_teleop_stop.call_async(self.call_teleop_stop_req)
        rclpy.spin_until_future_complete(self.node, resp)
        wx.CallAfter(self.update_reply_show, resp.result())
        event.Skip()
        self.node_lock.release()
    
    def call_set_bool_common(self, event, client, request):
        btn=event.GetEventObject()
        check_list=['Servo On', 'Servo Off', 'Clear Fault']
        
        # Check servo state
        if btn.GetName()=='Servo On':
            servo_enabled=bool()
            if self.servo_state_lock.acquire():
                servo_enabled=self.servo_state
                self.servo_state_lock.release()
            if servo_enabled:
                resp=SetBool_Response()
                resp.success=False
                resp.message='Robot is already enabled'
                wx.CallAfter(self.update_reply_show, resp)
                event.Skip()
                return
        
        # Check fault state
        if btn.GetName()=='Clear Fault':
            fault_flag=bool()
            if self.fault_state_lock.acquire():
                fault_flag=self.fault_state
                self.fault_state_lock.release()
            if not fault_flag:
                resp=SetBool_Response()
                resp.success=False
                resp.message='There is no fault now'
                wx.CallAfter(self.update_reply_show, resp)
                event.Skip()
                return
        
        # Check if the button is in check list
        if btn.GetName() in check_list:
            self.show_message_dialog(btn.GetName(), client, request)
        else:
            try:
                self.node_lock.acquire()
                resp = client.call_async(request)
                rclpy.spin_until_future_complete(self.node, resp)
                wx.CallAfter(self.update_reply_show, resp.result())
                self.node_lock.release()
            except Exception as e:
                resp=SetBool_Response()
                resp.success=False
                resp.message='Press button'
                # 'no such service in simulation1'
                wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def thread_bg(self, msg, client, request):
        wx.CallAfter(self.show_dialog)
        if msg=='Servo Off':
            self.action_stop()
        time.sleep(1.0)
        try:
            self.node_lock.acquire()
            resp=client.call_async(request)
            rclpy.spin_until_future_complete(self.node, resp)
            wx.CallAfter(self.update_reply_show, resp.result())
            self.node_lock.release()
        except Exception as e:
            resp=SetBool_Response()
            resp.success=False
            resp.message='Press Button'
            # 'no such service in simulation2'
            wx.CallAfter(self.update_reply_show, resp)
        wx.CallAfter(self.destroy_dialog)

    # 20201201: add function for processing value to DO_btn
    def process_DO_btn(self,value):
        if self.DO_btn_lock.acquire():
            for i in range(0,8):
                tmp = (value >> (12 + i)) & 0x01
                self.DO_btn[i]=tmp
            self.DO_btn_lock.release()

    # 20201201: add function to read DO.
    def call_read_DO_command(self):
        try:
            client = self.call_read_do
            val = client.call_async(self.call_read_do_req)
            rclpy.spin_until_future_complete(self.node, val,  executor=None, timeout_sec=0.05)
            if val.done():
                self.process_DO_btn(val.result().digital_input)
        except Exception as e:
            resp=ElfinIODRead_Response()
            resp.digital_input=0x0000

    # 20201201: add function for processing value
    def process_DI_btn(self,value):
        if self.DI_show_lock.acquire():
            if value > 0:
                for i in range(0,8):
                    tmp = (value >> (i)) & 0x01
                    self.DI_show[i]=tmp
            else:
                self.DI_show = [0,0,0,0,0,0,0,0]
        self.DI_show_lock.release()
    
    # 20201201: add function to read DI.
    def call_read_DI_command(self):
        try:
            client = self.call_read_di
            val = client.call_async(self.call_read_di_req)
            rclpy.spin_until_future_complete(self.node, val,  executor=None, timeout_sec=0.05)
            if val.done():
                self.process_DI_btn(val.result().digital_input)
        except Exception as e:
            resp=ElfinIODRead_Response()
            resp.digital_input=0x0000

    # 20201202: add function to read DO and DI.
    def monitor_DO_DI(self):
        self.node_lock.acquire()
        self.call_read_DI_command()
        self.call_read_DO_command()
        self.node_lock.release()

    # 20201126: add function to write DO.
    def call_write_DO_command(self, event, marker, client):
        self.justification_DO_btn(marker)
        write_request = ElfinIODWrite_Request()
        request = 0
        try:
            self.DO_btn_lock.acquire()
            for i in range(0,8):
                request = request + self.DO_btn[i]*pow(2,i)
            write_request.digital_output = request <<12
            resp=client.call_async(write_request)
            self.DO_btn_lock.release()
        except Exception as e:
            self.DO_btn_lock.release()
            resp=ElfinIODWrite_Response()
            resp.success=False
            self.justification_DO_btn(marker)
            rp=SetBool_Response()
            rp.success=False
            rp.message='no such service for DO control'
            wx.CallAfter(self.update_reply_show, rp)

    # 20201127: add justification to DO_btn
    def justification_DO_btn(self,marker):
        self.DO_btn_lock.acquire()
        if 0 == self.DO_btn[marker]:
            self.DO_btn[marker] = 1
        else:
             self.DO_btn[marker] = 0
        self.DO_btn_lock.release()

    # 20201201: add function to set DO_btn colour
    def set_DO_btn_colour(self):
        self.DO_btn_lock.acquire()
        for i in range(0,4):
            if 0 == self.DO_btn[i]:
                self.DO_btn_display[i].SetBackgroundColour(wx.NullColour)
            else:
                self.DO_btn_display[i].SetBackgroundColour(wx.Colour(200,225,200))
        self.DO_btn_lock.release()

    # 20201201: add function to set DI_show colour
    def set_DI_show_colour(self):
        self.DI_show_lock.acquire()
        for i in range(0,4):
            if 0 == self.DI_show[i]:
                self.DI_display[i].SetBackgroundColour(wx.NullColour)
            else:
                self.DI_display[i].SetBackgroundColour(wx.Colour(200,225,200))
        self.DI_show_lock.release()

    # 20201207: add function to set LED colour
    def set_LED_show_colour(self):
        self.DO_btn_lock.acquire()
        for i in range(4,8):
            if 0 == self.DO_btn[i]:
                self.LED_display[i-4].SetBackgroundColour(wx.NullColour)
            else:
                self.LED_display[i-4].SetBackgroundColour(wx.Colour(200,225,200))
        self.DO_btn_lock.release()

    # 20201207: add function to set End_btn colour
    def set_End_btn_colour(self):
        self.DI_show_lock.acquire()
        for i in range(4,8):
            if 0 == self.DI_show[i]:
                png=wx.Image(self.btn_path+'/btn_icon/End_btn'+str(i-4)+'_low.png',wx.BITMAP_TYPE_PNG)
                self.End_btn_display[i-4].SetBitmap(wx.Bitmap(png))
            else:
                png=wx.Image(self.btn_path+'/btn_icon/End_btn'+str(i-4)+'_high.png',wx.BITMAP_TYPE_PNG)
                self.End_btn_display[i-4].SetBitmap(wx.Bitmap(png))
        self.DI_show_lock.release()

    def set_color(self):
        wx.CallAfter(self.set_DO_btn_colour)
        wx.CallAfter(self.set_DI_show_colour)
        wx.CallAfter(self.set_LED_show_colour)
        wx.CallAfter(self.set_End_btn_colour)
    
    def show_message_dialog(self, message, cl, rq):
        msg='executing ['+message+']'
        self.dlg_label.SetLabel(msg)
        lable_size=[]
        lable_size.append(self.dlg_label.GetSize()[0])
        lable_size.append(self.dlg_label.GetSize()[1])
        self.dlg.SetSize((lable_size[0]+30, lable_size[1]+30))
        t=threading.Thread(target=self.thread_bg, args=(message, cl, rq,))
        t.start()
        
    def show_dialog(self):
        self.dlg.SetPosition((self.GetPosition()[0]+250,
                              self.GetPosition()[1]+250))
        self.dlg.ShowModal()
        
    def destroy_dialog(self):
        self.dlg.EndModal(0)
        
    def closewindow(self,event):
        pass
    
    def show_set_links_dialog(self, evt):
        self.sld_ref_link_show.SetValue(self.ref_link_name)
        self.sld_end_link_show.SetValue(self.end_link_name)
        self.set_links_dlg.SetPosition((self.GetPosition()[0]+150,
                                        self.GetPosition()[1]+250))
        self.set_links_dlg.ShowModal()
    
    def update_ref_link(self, evt):
        request=SetString_Request()
        request.data=self.sld_ref_link_show.GetValue()
        resp=self.call_set_ref_link.call_async(request)
        rclpy.spin_until_future_complete(self.node, resp)
        wx.CallAfter(self.update_reply_show, resp.result())
    
    def update_end_link(self, evt):
        request=SetString_Request()
        request.data=self.sld_end_link_show.GetValue()
        resp=self.call_set_end_link.call_async(request)
        rclpy.spin_until_future_complete(self.node, resp)
        wx.CallAfter(self.update_reply_show, resp.result())
    
    def updateDisplay(self, msg):      
        for i in range(len(self.js_display)):
            self.js_display[i].SetValue(msg[i])
        for i in range(len(self.ps_display)):
            self.ps_display[i].SetValue(msg[i+6])
            
        if self.ref_link_lock.acquire():
            ref_link=self.ref_link_name
            self.ref_link_lock.release()
        
        if self.end_link_lock.acquire():
            end_link=self.end_link_name
            self.end_link_lock.release()
        
        self.ref_link_show.SetValue(ref_link)
        self.end_link_show.SetValue(end_link)
    
    def update_reply_show(self,msg):
        if msg.success:
            self.reply_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.reply_show.SetValue(msg.message)
        else:
            self.reply_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.reply_show.SetValue(msg.message)# msg.message
            
    def update_servo_state(self, msg):
        if msg.data:
            self.servo_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.servo_state_show.SetValue('Enabled')
        else:
            self.servo_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.servo_state_show.SetValue('Disabled')
    
    def update_fault_state(self, msg):
        if msg.data:
            self.fault_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.fault_state_show.SetValue('Warning')
        else:
            self.fault_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.fault_state_show.SetValue('No Fault')
        
    def update_velocity_scaling_show(self, msg):
        self.velocity_setting_show.SetValue(str(round(msg, 2)*100)+'%') # 20201127: change the show format
    
    
    def js_call_back(self, data):
        while not rclpy.is_shutdown():
            try:
                self.listener.waitForTransform(self.group.get_planning_frame(),
                                               self.group.get_end_effector_link(),
                                               rclpy.Time(0), rclpy.Duration(100))
                (xyz,qua) = self.listener.lookupTransform(self.group.get_planning_frame(), 
                                                        self.group.get_end_effector_link(), 
                                                        rclpy.Time(0))
                break
            except (LookupException, ConnectivityException, ExtrapolationException):
            
                continue
        rpy=tf2_ros.transformations.euler_from_quaternion(qua)
        
        for i in range(len(data.position)):
            self.key.append(str(round(data.position[i]*180/math.pi, 2)))
            
        self.key.append(str(round(xyz[0]*1000, 2)))
        self.key.append(str(round(xyz[1]*1000, 2)))
        self.key.append(str(round(xyz[2]*1000, 2)))
        
        self.key.append(str(round(rpy[0]*180/math.pi, 2)))
        self.key.append(str(round(rpy[1]*180/math.pi, 2)))
        self.key.append(str(round(rpy[2]*180/math.pi, 2)))
        
        wx.CallAfter(self.updateDisplay, self.key)
        self.key=[]
    
    def monitor_status(self):
        self.key=[]
        t = geometry_msgs.msg.TransformStamped()
        current_joint_values=self.current_joint_val
        for i in range(len(current_joint_values)):
            self.key.append(str(round(current_joint_values[i]*180/math.pi, 2)))
        
        if self.ref_link_lock.acquire():
            ref_link=self.ref_link_name
            self.ref_link_lock.release()
        
        if self.end_link_lock.acquire():
            end_link=self.end_link_name
            self.end_link_lock.release()
        try:
            t = self.tfBuffer.lookup_transform(ref_link, end_link, rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            xyz = [x,y,z]
            
            eff_rpy = transforms3d.euler.quat2euler([t.transform.rotation.w,t.transform.rotation.x
                                ,t.transform.rotation.y,t.transform.rotation.z],"sxyz")
            rpy = [eff_rpy[0],eff_rpy[1],eff_rpy[2]]
            self.key.append(str(round(xyz[0]*1000, 2)))
            self.key.append(str(round(xyz[1]*1000, 2)))
            self.key.append(str(round(xyz[2]*1000, 2)))
            
            self.key.append(str(round(rpy[0]*180/math.pi, 2)))
            self.key.append(str(round(rpy[1]*180/math.pi, 2)))
            self.key.append(str(round(rpy[2]*180/math.pi, 2)))
            wx.CallAfter(self.updateDisplay, self.key)
        except Exception as e:
            self.node.get_logger().info('Get TF2 State Error...')
            
    def servo_state_cb(self, data):
        if self.servo_state_lock.acquire():
            self.servo_state=data.data
            self.servo_state_lock.release()
        wx.CallAfter(self.update_servo_state, data)
    
    def fault_state_cb(self, data):
        if self.fault_state_lock.acquire():
            self.fault_state=data.data
            self.fault_state_lock.release()
        wx.CallAfter(self.update_fault_state, data)
    
    def ref_link_name_cb(self, data):
        if self.ref_link_lock.acquire():
            self.ref_link_name=data.data
            self.ref_link_lock.release()
    
    def end_link_name_cb(self, data):
        if self.end_link_lock.acquire():
            self.end_link_name=data.data
            self.end_link_lock.release()
        
    def listen(self):
        self.gui_node.create_subscription(Bool, '/enable_state', self.servo_state_cb, 10)
        self.gui_node.create_subscription(Bool, '/fault_state', self.fault_state_cb, 10)
        self.gui_node.create_subscription(String, '/reference_link_name', self.ref_link_name_cb, 10)
        self.gui_node.create_subscription(String, '/end_link_name', self.end_link_name_cb, 10)
        self.gui_node.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.gui_node.create_timer(0.2, self.monitor_status)
        self.gui_node.create_timer(0.2, self.set_color)
        if not self.use_fake_robot:
            self.gui_node.create_timer(0.2, self.monitor_DO_DI)
        else:
            pass
       
        self.elfin_gui_executor = MultiThreadedExecutor()
        self.elfin_gui_executor.add_node(self.gui_node)
        self.elfin_gui_executor.add_node(self.node)
        spin_thread = threading.Thread(target=self.elfin_gui_executor.spin)
        spin_thread.setDaemon(True)
        spin_thread.start()
  
if __name__=='__main__':  
    app=wx.App(False)  
    myframe=MyFrame(parent=None,id=-1) 
    myframe.Show(True)
    myframe.listen()
    app.MainLoop()
