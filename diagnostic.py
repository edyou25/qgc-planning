def diagnostic_thread():
    """诊断线程，用于检查连接状态和消息收发情况"""
    global connections
    
    print("启动诊断线程...")
    add_status_message("启动诊断功能")
    
    last_check_time = time.time()
    check_interval = 10  # 每10秒检查一次
    
    while True:
        try:
            current_time = time.time()
            if current_time - last_check_time >= check_interval:
                print("\n=== 连接诊断信息 ===")
                
                # 检查所有连接
                for i, conn in enumerate(connections):
                    try:
                        # 获取连接信息
                        local_addr = conn.address if hasattr(conn, 'address') else "未知"
                        target_sys = conn.target_system if hasattr(conn, 'target_system') else "未设置"
                        target_comp = conn.target_component if hasattr(conn, 'target_component') else "未设置"
                        
                        print(f"连接 {i+1}: 端口={conn.port}, 目标系统={target_sys}, 目标组件={target_comp}")
                        
                        # 尝试发送心跳检查消息
                        if hasattr(conn, 'mav'):
                            try:
                                # 发送参数请求作为探测
                                conn.mav.param_request_list_send(
                                    conn.target_system if hasattr(conn, 'target_system') else 1,
                                    conn.target_component if hasattr(conn, 'target_component') else 1
                                )
                                print(f"  - 已发送参数请求探测")
                            except Exception as e:
                                print(f"  - 发送探测消息失败: {e}")
                        
                        # 尝试接收任何消息
                        msg = conn.recv_match(blocking=False)
                        if msg:
                            print(f"  - 收到消息类型: {msg.get_type()}")
                        else:
                            print(f"  - 未收到任何消息")
                            
                            # 特别检查QGC配置
                            if i == 0:  # 主连接
                                print("\n可能的问题排查:")
                                print("1. 确认QGC已正确配置通信链接:")
                                print(f"   - 主机: {WINDOWS_IP}, 端口: {QGC_PORT}")
                                print("2. 检查QGC中的通信链接是否已连接")
                                print("3. 确认QGC已连接到飞行控制器")
                                print("4. 尝试在QGC中更新连接设置")
                                print("5. 确保防火墙未阻止UDP通信")
                                add_status_message("未检测到消息传输，请检查QGC配置")
                    except Exception as e:
                        print(f"检查连接 {i+1} 时出错: {e}")
                
                # 检查当前位置信息
                if current_position['lat'] is not None:
                    print(f"\n当前位置: 纬度={current_position['lat']:.6f}, 经度={current_position['lon']:.6f}, 高度={current_position['alt']:.1f}m")
                else:
                    print("\n未接收到位置信息")
                
                # 更新检查时间
                last_check_time = current_time
                print("=== 诊断信息结束 ===\n")
                
            time.sleep(1)
        except Exception as e:
            print(f"诊断线程错误: {e}")
            time.sleep(5)
