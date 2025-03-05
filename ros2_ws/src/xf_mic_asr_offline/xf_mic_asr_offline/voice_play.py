#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/21
import os
#from ament_index_python.packages import get_package_share_directory
#wav_dir = get_package_share_directory('xf_mic_asr_offline')
#wav_path = os.path.join(wav_dir, 'share/xf_ring_mic_asr_offline/feedback_voice')
wav_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline/feedback_voice'

def get_path(f, language='Chinese'):
    if language == 'Chinese':
        return os.path.join(wav_path, f + '.wav')
    else:    
        return os.path.join(wav_path, 'english', f + '.wav')

def play(voice, volume=80, language='Chinese'):
    try:
        # os.system('amixer -q -D pulse set Master {}%'.format(volume))
        os.system('play -q ' + get_path(voice, language))
    except BaseException as e:
        print('error', e)

if __name__ == '__main__':
    play('ok')
    play('running', language="English")

