#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/06/01
# @author:aiden
# PID追踪

class PIDTrack:
    def __init__(self, pid_1, pid_2, range_1, range_2, rl_init=0, ud_init=0):
        self.rl_dis = rl_init
        self.ud_dis = ud_init
        
        self.pid_1 = pid_1
        self.pid_2 = pid_2

        self.range_1 = range_1
        self.range_2 = range_2

    def update_position(self, rl_dis, ud_dis):
        self.rl_dis = rl_dis
        self.ud_dis = ud_dis

    def clear(self):
        self.pid_1.clear()
        self.pid_2.clear()

    def track(self, x, y, target_x, target_y):
        self.pid_1.SetPoint = target_x 
        self.pid_1.update(x)
        self.rl_dis += self.pid_1.output
        
        if self.rl_dis < self.range_1[0]:
            self.rl_dis = self.range_1[0]
        if self.rl_dis > self.range_1[1]:
            self.rl_dis = self.range_1[1]
        
        self.pid_2.SetPoint = target_y
        self.pid_2.update(y)
        self.ud_dis += self.pid_2.output
        
        if self.ud_dis < self.range_2[0]:
            self.ud_dis = self.range_2[0]
        if self.ud_dis > self.range_2[1]:
            self.ud_dis = self.range_2[1]

        return self.rl_dis, self.ud_dis
