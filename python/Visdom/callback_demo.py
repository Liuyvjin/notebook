#!/usr/bin/env python3
# Visdom 回调函数示例

from visdom import Visdom
import time
import numpy as np

try:
    viz = Visdom(env='test_main')  # 下划线表示层级化, 本例会生成 test/test_main 二级环境

    assert viz.check_connection(timeout_seconds=3), \
        'No connection could be formed quickly'

    """ 1. 图片回调示例: 应用事件 KeyPress, Click """
    color = 0.5
    image = np.full([3, 256, 256], 1, dtype=float)
    opts = dict(title='image-demo', caption='Press arrows to alter color, Click to print')
    image_win_id = viz.image(image*color, opts=opts, win='win01')  # 指定窗口id为 'win01'

    def image_callback(event):  # 回调函数
        global color  # 全局变量
        if event['event_type'] == 'KeyPress':  # 处理左/右按键
            if event['key'] == 'ArrowRight':
                color = min(color + 0.1, 1)
            if event['key'] == 'ArrowLeft':
                color = max(color - 0.1, 0)
            viz.image(image*color, opts=opts, win=image_win_id)  # 向之前的窗口绘制指定颜色
        elif event['event_type'] == 'Click':  # 目前 click 事件有bug
            print( event['image_coord'])

    viz.register_event_handler(image_callback, image_win_id)  # 将回调函数与窗口id绑定
    # visdom对象内部保存有一个字典: event_handlers: {'win_id': [func1, func2, ...], ...}
    # 该函数其实是执行: viz.event_handlers['win_id'].append(callbackfunc)
    # 相对应的有删除回调函数的接口: clear_event_handler, 作用是 viz.event_handlers['win_id'] = []


    """ 2. 具有回调函数的文字窗口示例: 应用事件 KeyPress """
    txt = 'This is a write demo notepad. Type below. [Delete]: clears text.<br>'
    # 创建一个 text 窗口并返回 id
    text_window_id = viz.text(txt, win='win02', opts={'title': 'text-demo'})

    def type_callback(event):  # 输入回调函数
        if event['event_type'] == 'KeyPress':  # 键盘响应
            curr_txt = event['pane_data']['content']  # 读取窗口中已输入的内容
            if event['key'] == 'Enter':  # 根据按键将新内容写入 curr_txt
                curr_txt += '<br>'  # 换行
            elif event['key'] == 'Backspace':
                curr_txt = curr_txt[:-1]
            elif event['key'] == 'Delete':
                curr_txt = txt
            elif len(event['key']) == 1:
                curr_txt += event['key']
            viz.text(curr_txt, win=text_window_id, append=False, opts={'title': 'text-demo'})  # 写入到指定窗口

    viz.register_event_handler(type_callback, text_window_id)


    """ 3. 属性窗口示例: 应用事件 PropertyUpdate """
    properties = [
        {'type': 'text', 'name': 'Text input', 'value': 'initial'},  # 各种类型的属性栏
        {'type': 'number', 'name': 'Number input', 'value': '12'},
        {'type': 'button', 'name': 'Button', 'value': 'Start'},
        {'type': 'checkbox', 'name': 'Checkbox', 'value': True},
        {'type': 'select', 'name': 'Select', 'value': 1, 'values': ['Red', 'Green', 'Blue']},
    ]
    # 新建属性窗口, 并返回窗口 id
    properties_window_id = viz.properties(properties, win='win03', opts={'title': 'prop-demo'})

    def properties_callback(event):
        if event['event_type'] == 'PropertyUpdate':  # 属性更新事件
            prop_id = event['propertyId']  # 属性 id, 为定义窗口时传入的 properties 列表中相应属性的 idx
            value = event['value']  # 新的属性值
            if prop_id in [0, 1]:  # 字符属性 和 数字属性 无需在本地字典中更新, 更新也不会影响显示
                new_value = value + '_updated'  # 可以什么也不干
                new_value = value + '0'
            elif prop_id == 2:  # Button属性 名称可以在本地字典修改
                new_value = 'Stop' if properties[prop_id]['value'] == 'Start' else 'Start'
            else:    # 3 checkbox, 4 select属性, 需要本地修改, 并重新写入
                new_value = value
            properties[prop_id]['value'] = new_value  # 修改字典
            viz.properties(properties, win=properties_window_id, opts={'title': 'prop-demo'})  # 将新字典写入, 其中 text 和 number 不会起作用
            viz.text("Updated: {} => {}".format(properties[prop_id]['name'], str(value)),  # 将更新信息写入text win
                     win=text_window_id, append=True)

    viz.register_event_handler(properties_callback, properties_window_id)

    input('Waiting for callbacks, press enter to quit.')  # 中断阻塞等待

except BaseException as e:
    print(
        "The visdom experienced an exception while running: {}\n".format(repr(e))
    )