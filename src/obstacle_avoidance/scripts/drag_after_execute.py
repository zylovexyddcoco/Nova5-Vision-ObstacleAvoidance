#!/usr/bin/env python3
"""
Dobot拖拽模式控制器 - 改进版
- 自动重连
- 错误恢复
- 使用ROS2服务作为备选

使用方法：
  python3 drag_controller.py

命令：
  d - Enter drag mode (进入拖拽模式)
  s - Stop drag mode (退出拖拽模式)  
  e - Enable robot (使能机械臂)
  m - Get robot mode (获取状态)
  c - Clear error (清除报警)
  r - Reconnect (重新连接)
  q - Quit (退出程序)
"""
import socket
import time
import sys

class DragModeController:
    def __init__(self, robot_ip="192.168.5.1"):
        self.robot_ip = robot_ip
        self.dashboard_port = 29999
        self.dashboard_socket = None
        self.connected = False
        
        self.mode_names = {
            1: 'INIT (初始化)',
            2: 'BRAKE_OPEN (抱闸松开)',
            3: 'POWER_OFF (未上电)',
            4: 'DISABLED (未使能)',
            5: 'ENABLE (使能空闲)',
            6: 'BACKDRIVE (拖拽模式)',
            7: 'RUNNING (运行中)',
            8: 'RECORDING (录制中)',
            9: 'ERROR (报警)',
            10: 'PAUSE (暂停)',
            11: 'JOG (点动中)'
        }
    
    def connect(self):
        """连接到机械臂Dashboard端口"""
        try:
            # 关闭旧连接
            if self.dashboard_socket:
                try:
                    self.dashboard_socket.close()
                except:
                    pass
            
            self.dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard_socket.settimeout(5)
            self.dashboard_socket.connect((self.robot_ip, self.dashboard_port))
            
            # 读取欢迎信息
            welcome = self.dashboard_socket.recv(1024).decode('utf-8')
            print(f'[INFO] Connected to {self.robot_ip}:{self.dashboard_port}')
            print(f'[INFO] Welcome: {welcome.strip()}')
            self.connected = True
            return True
        except Exception as e:
            print(f'[ERROR] Connection failed: {e}')
            self.connected = False
            return False
    
    def reconnect(self):
        """重新连接"""
        print('[INFO] Reconnecting...')
        return self.connect()
    
    def send_command(self, cmd):
        """发送命令并获取响应，带自动重连"""
        max_retries = 2
        
        for attempt in range(max_retries):
            try:
                if not self.connected:
                    if not self.reconnect():
                        continue
                
                # 发送命令
                full_cmd = cmd if cmd.endswith('\n') else cmd + '\n'
                self.dashboard_socket.send(full_cmd.encode('utf-8'))
                print(f'[SEND] {cmd.strip()}')
                
                # 接收响应
                time.sleep(0.1)
                response = self.dashboard_socket.recv(1024).decode('utf-8')
                print(f'[RECV] {response.strip()}')
                
                # 解析ErrorID
                if response:
                    parts = response.split(',')
                    if len(parts) > 0:
                        try:
                            error_id = int(parts[0])
                            return error_id == 0, response, error_id
                        except ValueError:
                            pass
                return False, response, -1
                
            except (BrokenPipeError, ConnectionResetError, socket.error) as e:
                print(f'[WARN] Connection error: {e}, reconnecting...')
                self.connected = False
                if attempt < max_retries - 1:
                    time.sleep(0.5)
                    self.reconnect()
            except Exception as e:
                print(f'[ERROR] Command error: {e}')
                return False, str(e), -1
        
        return False, "Connection failed after retries", -1
    
    def get_robot_mode(self):
        """获取机器人状态"""
        success, response, _ = self.send_command('RobotMode()')
        if success:
            try:
                parts = response.split('{')
                if len(parts) > 1:
                    value = parts[1].split('}')[0]
                    mode = int(value)
                    mode_name = self.mode_names.get(mode, f'Unknown({mode})')
                    print(f'[STATUS] Robot Mode: {mode} - {mode_name}')
                    return mode
            except:
                pass
        return -1
    
    def enter_drag_mode(self):
        """进入拖拽模式"""
        print('[ACTION] Entering drag mode...')
        success, _, error_id = self.send_command('StartDrag()')
        if success:
            print('=' * 50)
            print('✓ Now in DRAG MODE')
            print('  You can manually move the robot arm')
            print('  Press "s" when done to exit drag mode')
            print('=' * 50)
        else:
            print(f'[ERROR] Failed to enter drag mode (error: {error_id})')
            if error_id == -1:
                print('  Tip: Robot may need to be enabled first (press "e")')
        return success
    
    def exit_drag_mode(self):
        """退出拖拽模式"""
        print('[ACTION] Exiting drag mode...')
        success, _, error_id = self.send_command('StopDrag()')
        if success:
            print('✓ Exited drag mode')
            print('  Remember to enable robot (press "e") before motion commands')
        else:
            print(f'[ERROR] Failed to exit drag mode (error: {error_id})')
        return success
    
    def enable_robot(self):
        """使能机械臂"""
        print('[ACTION] Enabling robot...')
        success, _, _ = self.send_command('EnableRobot()')
        if success:
            print('✓ Robot enabled')
            # 验证模式
            time.sleep(0.3)
            mode = self.get_robot_mode()
            if mode == 5:
                print('✓ Robot is ready (mode=5 ENABLE)')
            elif mode == 6:
                print('[WARN] Still in drag mode! Run "s" to exit first')
        return success
    
    def disable_robot(self):
        """下使能机械臂"""
        print('[ACTION] Disabling robot...')
        success, _, _ = self.send_command('DisableRobot()')
        return success
    
    def clear_error(self):
        """清除报警"""
        print('[ACTION] Clearing errors...')
        success, _, _ = self.send_command('ClearError()')
        if success:
            print('✓ Errors cleared')
        return success
    
    def get_pose(self):
        """获取当前位姿"""
        success, response, _ = self.send_command('GetPose()')
        if success:
            try:
                parts = response.split('{')
                if len(parts) > 1:
                    values = parts[1].split('}')[0]
                    print(f'[POSE] {values}')
            except:
                pass
        return success
    
    def close(self):
        """关闭连接"""
        if self.dashboard_socket:
            try:
                self.dashboard_socket.close()
            except:
                pass
            print('[INFO] Connection closed')
    
    def run_interactive(self):
        """交互式运行"""
        print("\n" + "=" * 60)
        print("       Dobot Drag Mode Controller (Improved)")
        print("=" * 60)
        print("Commands:")
        print("  d - Enter drag mode (进入拖拽模式)")
        print("  s - Stop drag mode (退出拖拽模式)")
        print("  e - Enable robot (使能机械臂)")
        print("  m - Get robot mode (获取状态)")
        print("  p - Get current pose (获取位姿)")
        print("  c - Clear error (清除报警)")
        print("  r - Reconnect (重新连接)")
        print("  q - Quit (退出程序)")
        print("=" * 60)
        print("\nTypical workflow after MoveIt execute:")
        print("  1. Press 'd' to enter drag mode")
        print("  2. Manually drag robot back to start position")
        print("  3. Press 's' to exit drag mode")
        print("  4. Press 'e' to enable robot")
        print("  5. Ready for next planning")
        print("=" * 60 + "\n")
        
        # 首先检查当前状态
        print("[INFO] Checking current robot state...")
        self.get_robot_mode()
        
        try:
            while True:
                cmd = input("\nEnter command (d/s/e/m/p/c/r/q): ").strip().lower()
                
                if cmd == 'd':
                    self.enter_drag_mode()
                elif cmd == 's':
                    self.exit_drag_mode()
                elif cmd == 'e':
                    self.enable_robot()
                elif cmd == 'm':
                    self.get_robot_mode()
                elif cmd == 'p':
                    self.get_pose()
                elif cmd == 'c':
                    self.clear_error()
                elif cmd == 'r':
                    self.reconnect()
                elif cmd == 'q':
                    # 退出前检查状态
                    mode = self.get_robot_mode()
                    if mode == 6:
                        print("\n[WARN] Robot is still in drag mode!")
                        confirm = input("Exit drag mode before quitting? (y/n): ").strip().lower()
                        if confirm == 'y':
                            self.exit_drag_mode()
                            self.enable_robot()
                    print("Exiting...")
                    break
                elif cmd == '':
                    continue
                else:
                    print(f"Unknown command: '{cmd}'")
                    print("Valid commands: d, s, e, m, p, c, r, q")
                    
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user")
            # 检查是否需要退出拖拽模式
            try:
                mode = self.get_robot_mode()
                if mode == 6:
                    print("[WARN] Robot is still in drag mode!")
                    self.exit_drag_mode()
                    self.enable_robot()
            except:
                pass
        finally:
            self.close()


def main():
    robot_ip = "192.168.5.1"
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    
    print(f"Connecting to robot at {robot_ip}...")
    
    controller = DragModeController(robot_ip)
    
    if controller.connect():
        controller.run_interactive()
    else:
        print("[ERROR] Failed to connect to robot")
        print("Make sure:")
        print("  1. Robot is powered on")
        print("  2. Network connection is correct")
        print(f"  3. Robot IP is {robot_ip}")
        sys.exit(1)


if __name__ == '__main__':
    main()
