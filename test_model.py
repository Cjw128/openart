# ======================================================================
# TFLite 模型测试脚本
# 用途: 单独测试模型加载和推理，排查 main.py 死机原因
# ======================================================================

import sensor, time, gc

# ======================================================================
# 配置
# ======================================================================
MODEL_PATH = '/sd/dataset_25000_blur_0.30.tflite'
MODEL_INPUT_SCALE = 0.75   # 与 main.py 一致
SCORE_THRESHOLD = 0.40
LABELS = ['bear', 'ball', 'bag']

# ======================================================================
# 摄像头初始化
# ======================================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240
sensor.set_framerate(30)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=1000)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=1200)
sensor.set_auto_gain(False, gain_db=0)

clock = time.clock()

# ======================================================================
# 步骤1: 打印加载前内存
# ======================================================================
gc.collect()
mem_before = gc.mem_free()
print("=" * 50)
print("TFLite 模型测试脚本")
print("=" * 50)
print("模型路径:", MODEL_PATH)
print("输入缩放:", MODEL_INPUT_SCALE)
print("加载前可用内存: {} bytes ({:.1f} KB)".format(mem_before, mem_before / 1024))
print()

# ======================================================================
# 步骤2: 尝试加载模型
# ======================================================================
print(">>> 正在加载模型...")
net = None
tf = None
load_ok = False
load_ms = 0

try:
    import tf as _tf
    tf = _tf
    gc.collect()
    t0 = time.ticks_ms()
    net = tf.load(MODEL_PATH)
    load_ms = time.ticks_diff(time.ticks_ms(), t0)
    load_ok = True
    gc.collect()
    mem_after = gc.mem_free()
    print(">>> 模型加载成功!")
    print("    耗时: {} ms".format(load_ms))
    print("    加载后可用内存: {} bytes ({:.1f} KB)".format(mem_after, mem_after / 1024))
    print("    模型占用内存: {} bytes ({:.1f} KB)".format(mem_before - mem_after, (mem_before - mem_after) / 1024))
except MemoryError as e:
    print(">>> [内存不足] 模型加载失败:", e)
    print("    可用内存仅 {:.1f} KB，模型太大".format(mem_before / 1024))
except OSError as e:
    print(">>> [文件错误] 模型加载失败:", e)
    print("    请检查 SD 卡是否插入，路径是否正确:", MODEL_PATH)
except Exception as e:
    print(">>> [未知错误] 模型加载失败:", e)

print()

# ======================================================================
# 步骤3: 如果加载成功，运行推理测试 (50帧)
# ======================================================================
if load_ok and net is not None:
    print(">>> 开始推理测试 (50帧)，按复位键退出...")
    print("-" * 50)

    frame_count = 0
    detect_count = 0
    error_count = 0
    last_print = time.ticks_ms()

    while frame_count < 50:
        clock.tick()
        frame_count += 1

        img = sensor.snapshot()

        # 缩放图像（与 main.py 一致）
        detect_img = img
        if MODEL_INPUT_SCALE != 1.0:
            try:
                detect_img = img.copy(MODEL_INPUT_SCALE, 1)
            except Exception as e:
                print("[帧{}] img.copy 失败: {}".format(frame_count, e))
                error_count += 1
                continue

        # 运行推理
        try:
            results = tf.detect(net, detect_img)
            for obj in results:
                x1, y1, x2, y2, label, score = obj
                label = int(label)
                score = float(score)
                if score >= SCORE_THRESHOLD:
                    detect_count += 1
                    lname = LABELS[label] if label < len(LABELS) else str(label)
                    # 在原图上画框
                    rx = int(x1 * img.width())
                    ry = int(y1 * img.height())
                    rw = int((x2 - x1) * img.width())
                    rh = int((y2 - y1) * img.height())
                    img.draw_rectangle((rx, ry, rw, rh), color=(255, 0, 0), thickness=2)
                    img.draw_string(rx, max(0, ry - 12),
                                    "{} {:.2f}".format(lname, score),
                                    color=(255, 255, 0), scale=1)
        except Exception as e:
            print("[帧{}] 推理失败: {}".format(frame_count, e))
            error_count += 1

        if detect_img is not img:
            detect_img = None

        # 每秒打印一次状态
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) >= 1000:
            gc.collect()
            print("[帧{}/50] FPS={:.1f} 检测次数={} 错误={} 剩余内存={:.1f}KB".format(
                frame_count, clock.fps(), detect_count, error_count, gc.mem_free() / 1024))
            last_print = now

    # 测试结束汇总
    gc.collect()
    print()
    print("=" * 50)
    print("推理测试完成")
    print("  总帧数  : {}".format(frame_count))
    print("  检测次数: {}".format(detect_count))
    print("  错误次数: {}".format(error_count))
    print("  平均FPS : {:.1f}".format(clock.fps()))
    print("  剩余内存: {:.1f} KB".format(gc.mem_free() / 1024))
    print("=" * 50)

else:
    # 模型加载失败，继续跑摄像头确认硬件正常
    print(">>> 模型未加载，仅运行摄像头预览 (按复位键退出)...")
    while True:
        clock.tick()
        img = sensor.snapshot()
        img.draw_string(2, 2, "No model  FPS:{:.1f}".format(clock.fps()),
                        color=(255, 0, 0), scale=1)
