#!/usr/bin/env python3
# Viddom 基本示例

from visdom import Visdom
import argparse
import numpy as np
import math
import os.path
import time
import tempfile
import urllib
import matplotlib.pyplot as plt

def run_demo(viz):
    global input
    assert viz.check_connection(timeout_seconds=3), \
        'No connection could be formed quickly'   # 检查连接情况

    #--- 01. matplotlib demo:
    try:
        opt = {'title': '01_matplot'}  # 窗口命名
        plt.plot([1, 23, 2, 4])
        plt.ylabel('some numbers')
        viz.matplot(plt, win='win01', opts=opt)
    except BaseException as err:
        print('Skipped matplotlib example')
        print('Error message: ', err)

    #--- 02. image demo
    img_callback_win = viz.image(
        np.random.rand(3, 512, 256),
        opts={'title': '02_image_1', 'caption': 'Click me!'},
        win='win02_1'
    )

    img_coord_text = viz.text("Coords: ", opts={'title': '02_image_2'}, win='win02_2')

    def img_click_callback(event):  # 点击回调函数, 将点击位置输出到窗口中
        nonlocal img_coord_text
        if event['event_type'] != 'Click':
            return

        coords = "x: {}, y: {};".format(
            event['image_coord']['x'], event['image_coord']['y']
        )
        img_coord_text = viz.text(coords, win=img_coord_text, append=True)

    viz.register_event_handler(img_click_callback, img_callback_win)

    #--- 03. image demo save as jpg
    viz.image(
        np.random.rand(3, 512, 256),
        opts=dict(title='03_image_jpg', caption='How random as jpg.', jpgquality=50),
        win='win03'
    )

    #--- 04. images history demo 滑条切换图片
    viz.image(
        np.random.rand(3, 512, 256),
        win='win04',
        opts=dict(title='04_image_history', caption='First random', store_history=True),
    )
    viz.image(
        np.random.rand(3, 512, 256),
        win='win04',
        opts=dict(title='04_image_history', caption='Second random!', store_history=True),
    )

    #--- 05. grid of images 可用于绘制多张图片拼接
    viz.images(
        np.random.randn(20, 3, 64, 64),
        opts=dict(title='05_grid_images', caption='How random.'),
        win='win05'
    )

    #--- 06. scatter plots 散点图, 如何更新图的设置
    Y = np.random.rand(100)
    old_scatter = viz.scatter(
        X=np.random.rand(100, 2),
        Y=(Y[Y > 0] + 1.5).astype(int),  # 1.5~2.5 取int后为 1 或 2, 为 x 的两种label
        opts=dict(
            title='06_scatter',
            legend=['Didnt', 'Update'],  # 设置 lagend
            xtickmin=-50,
            xtickmax=50,
            xtickstep=0.5,
            ytickmin=-50,
            ytickmax=50,
            ytickstep=0.5,
            markersymbol='cross-thin-open',
        ),
        win = 'win06'
    )

    viz.update_window_opts(  # 更新图的配置
        win=old_scatter,
        opts=dict(
            title='06_scatter',
            legend=['Apples', 'Pears'],
            xtickmin=0,
            xtickmax=1,
            xtickstep=0.5,
            ytickmin=0,
            ytickmax=1,
            ytickstep=0.5,
            markersymbol='cross-thin-open',
        ),
    )

    #---07. 3d scatterplot with custom labels and ranges 带标签的3d散点图
    viz.scatter(
        X=np.random.rand(100, 3),
        Y=(Y + 1.5).astype(int),   # 1.5~2.5 取int后为 1 或 2, 为 x 的两种label
        opts=dict(
            title='07_3d_scatter',
            legend=['Men', 'Women'],
            markersize=5,
            xtickmin=0,
            xtickmax=2,
            xlabel='Arbitrary',
            xtickvals=[0, 0.75, 1.6, 2],
            ytickmin=0,
            ytickmax=2,
            ytickstep=0.5,
            ztickmin=0,
            ztickmax=1,
            ztickstep=0.5,
        ),
        win='win07'
    )

    #--- 08. 2D scatterplot with custom color per label and per point
    viz.scatter(  # 每个标签一种颜色
        X=np.random.rand(255, 2),
        Y=(np.random.rand(255) + 1.5).astype(int),
        opts=dict(
            title='08_scatter_label_color',
            markersize=10,
            markercolor=np.random.randint(0, 255, (2, 3,)),
            markerborderwidth=0,  # marker 边线宽度
        ),
        win='win08_1'
    )

    win = viz.scatter(  # 每个点一种颜色
        X=np.random.rand(255, 2),
        opts=dict(
            title='08_scatter_point_color_10_add_trace',
            markersize=10,
            markercolor=np.random.randint(0, 255, (255, 3,)),
        ),
        win='win08_2'
    )

    #--- 09. 判断 win 是否存在
    assert viz.win_exists(win), 'Created window marked as not existing'

    #--- 10. add new trace to scatter plot
    viz.scatter(
        X=np.random.rand(255),
        Y=np.random.rand(255),
        win=win,
        name='new_trace',
        update='new'
    )

    #--- 11. 2D scatter plot with text labels: 为每个点添加文字说明
    viz.scatter(
        X=np.random.rand(10, 2),
        Y=[1] * 5 + [2] * 3 + [3] * 2,  # 10个点共 3 类
        opts=dict(
            title='11_scatter_text_labels',
            legend=['A', 'B', 'C'],  # 添加lagend
            textlabels=['Label %d' % (i + 1) for i in range(10)]  # 每个点添加一个文字说明
        ),
        win='win11'
    )

    #--- 12. bar plots 条形统计图
    viz.bar(X=np.random.rand(20), opts=dict(title='12_bar_1'), win='win12_1')
    viz.bar(
        X=np.abs(np.random.rand(5, 3)),  # 五根柱子, 每根柱子分三种颜色
        opts=dict(
            title='12_bar_2',
            stacked=True,  # 三种颜色叠在一根柱上
            legend=['Facebook', 'Google', 'Twitter'],
            rownames=['2012', '2013', '2014', '2015', '2016']
        ),
        win='win12_2'
    )
    viz.bar(
        X=np.random.rand(4, 3),  # 12根柱, 分三种颜色
        opts=dict(
            title='12_bar_3',
            stacked=False,
            legend=['The Netherlands', 'France', 'United States']
        ),
        win='win12_3'
    )

    #--- 13. histogram 直方图
    viz.histogram(
        X=np.random.rand(10000),
        opts=dict(
            title='13_histogram',
            numbins=20,  # 直方图分20类
        ),
        win='win13'
    )

    #--- 14. heatmap 热力图
    viz.heatmap(
        X=np.outer(np.arange(1, 6), np.arange(1, 11)),  # 6 x 11
        opts=dict(
            title='14_heatmap',
            columnnames=['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j'],
            rownames=['y1', 'y2', 'y3', 'y4', 'y5'],
            colormap='Electric',
        ),
        win='win14'
    )

    #--- 15. contour 等高线图
    x = np.tile(np.arange(1, 101), (100, 1))
    y = x.transpose()
    X = np.exp((((x - 50) ** 2) + ((y - 50) ** 2)) / -(20.0 ** 2))
    viz.contour(
        X=X,
        opts=dict(title='15_contour', colormap='Viridis'),
        win='win15'
    )

    #--- 16. surface 三维曲面图
    viz.surf(
        X=X,
        opts=dict(title='16_surface', colormap='Hot'),
        win='win16'
    )

    #--- 17. line plots 折线图
    viz.line(
        Y=np.random.rand(10),  # 可以不指定 X
        opts=dict(title='17_line_1', showlegend=True),
        win='win17_1'
    )

    Y = np.linspace(-5, 5, 100)
    viz.line(  # 同时指定 Y 和 X
        Y=np.column_stack((Y * Y, np.sqrt(Y + 5))),  # 两条曲线 y=x^2 和 y=sqrt(x+5)
        X=np.column_stack((Y, Y)),
        opts=dict(title='17_line_2', markers=False),
        win='win17_2'
    )

    #--- 18. line using WebGL 在点非常多的时候可以用 WebGL, 但注意, 一个页面只能用一次
    webgl_num_points = 200000
    webgl_x = np.linspace(-1, 0, webgl_num_points)
    webgl_y = webgl_x**3
    viz.line(
        X=webgl_x, Y=webgl_y,
        opts=dict(title='18_{}_points_WebGL'.format(webgl_num_points), webgl=True),
        win="win18"
    )


    #--- 19. line updates 折线图更新
    win = viz.line(
        X=np.column_stack((np.arange(0, 10), np.arange(0, 10))),
        Y=np.column_stack((np.linspace(5, 10, 10),
                           np.linspace(5, 10, 10) + 5)),
        opts=dict(title='19_line_updates', showlegend=True),
        win='win19'
    )
    viz.line(
        X=np.column_stack((np.arange(10, 20), np.arange(10, 20))),
        Y=np.column_stack((np.linspace(5, 10, 10),
                           np.linspace(5, 10, 10) + 5)),  # 更新 1, 2 两条线
        win=win,
        update='append'  # 附加方式更新
    )
    viz.line(
        X=np.arange(21, 30),
        Y=np.arange(1, 10),
        win=win,
        name='2',  # 更新第二条线
        update='append'
    )
    viz.line(
        X=np.arange(1, 10),
        Y=np.arange(11, 20),
        win=win,
        name='3',
        update='append'
    )
    viz.line(X=None, Y=None, win=win, name='3', update='remove')  # 删除线
    viz.line(
        X=np.arange(1, 10),
        Y=np.arange(11, 20),
        win=win,
        name='4',
        update='insert'  # insert 模式, 创建一条新线
    )

    #--- 20. 指定不同线型
    win = viz.line(
        X=np.column_stack((
            np.arange(0, 10),
            np.arange(0, 10),
            np.arange(0, 10),
        )),
        Y=np.column_stack((
            np.linspace(5, 10, 10),
            np.linspace(5, 10, 10) + 5,
            np.linspace(5, 10, 10) + 10,
        )),
        opts={
            'title': '20_line_dash_types',
            'dash': np.array(['solid', 'dash', 'dashdot']),
            'showlegend': True,
            'linecolor': np.array([
                [0, 191, 255],
                [0, 191, 255],
                [255, 0, 0],
            ]),
        },
        win='win20'
    )

    viz.line(
        X=np.arange(0, 10),
        Y=np.linspace(5, 10, 10) + 15,
        win=win,
        name='4',
        update='insert',  # 插入一根红色点线
        opts={
            'linecolor': np.array([
                [255, 0, 0],
            ]),
            'dash': np.array(['dot']),
        }
    )

    #--- 21. Stack area plot
    Y = np.linspace(0, 4, 200)
    win = viz.line(
        Y=np.column_stack((np.sqrt(Y), np.sqrt(Y) + 2)),
        X=np.column_stack((Y, Y)),
        opts=dict(
            fillarea=True,
            showlegend=False,
            width=800,
            height=800,
            xlabel='Time',
            ylabel='Volume',
            ytype='log',
            title='21_Stacked_area_plot',
            marginleft=30,
            marginright=30,
            marginbottom=80,
            margintop=30,
        ),
        win='win21'
    )

    # Assure that the stacked area plot isn't giant
    viz.update_window_opts(
        win=win,
        opts=dict(
            width=300,
            height=300,
        ),
    )

    #--- 22. boxplot
    X = np.random.rand(100, 2)
    X[:, 1] += 2
    viz.boxplot(
        X=X,
        opts=dict(title='22_boxplot', legend=['Men', 'Women']),
        win='win22'
    )

    #--- 23. stemplot
    Y = np.linspace(0, 2 * math.pi, 70)
    X = np.column_stack((np.sin(Y), np.cos(Y)))
    viz.stem(
        X=X, Y=Y,
        opts=dict(title='23_stemplot', legend=['Sine', 'Cosine']),
        win='win23'
    )

    #--- 24. quiver plot 流场图
    X = np.arange(0, 2.1, .2)
    Y = np.arange(0, 2.1, .2)
    X = np.broadcast_to(np.expand_dims(X, axis=1), (len(X), len(X)))
    Y = np.broadcast_to(np.expand_dims(Y, axis=0), (len(Y), len(Y)))
    U = np.multiply(np.cos(X), Y)
    V = np.multiply(np.sin(X), Y)
    viz.quiver(
        X=U,
        Y=V,
        opts=dict(title='24_quiver', normalize=0.9),
        win='win24'
    )

    #--- 25. pie chart 扇形统计图
    X = np.asarray([19, 26, 55])
    viz.pie(
        X=X,
        opts=dict(title='25_pie_chart', legend=['Residential', 'Non-Residential', 'Utility']),
        win='win25'
    )

    #--- 26. scatter plot example with various type of updates 散点图更新
    colors = np.random.randint(0, 255, (2, 3,))
    win = viz.scatter(
        X=np.random.rand(255, 2),
        Y=(np.random.rand(255) + 1.5).astype(int),
        opts=dict(
            title='26_scatter_updates',
            markersize=10,
            markercolor=colors,
            legend=['1', '2']
        ),
        win='win26'
    )

    viz.scatter(
        X=np.random.rand(255),
        Y=np.random.rand(255),
        opts=dict(
            markersize=10,
            markercolor=colors[0].reshape(-1, 3),
        ),
        name='1',  # 添加进类别 1
        update='append',
        win=win)

    #--- 27. mesh plot
    x = [0, 0, 1, 1, 0, 0, 1, 1]
    y = [0, 1, 1, 0, 0, 1, 1, 0]
    z = [0, 0, 0, 0, 1, 1, 1, 1]
    X = np.c_[x, y, z]
    i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2]
    j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3]
    k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6]
    Y = np.c_[i, j, k]
    viz.mesh(X=X, Y=Y, opts=dict(title='27_mesh', opacity=0.5), win='win27')

    #--- 28. double y axis plot
    # X = np.arange(20)
    # Y1 = np.random.randint(0, 20, 20)
    # Y2 = np.random.randint(0, 20, 20)
    # viz.dual_axis_lines(X, Y1, Y2, opts=dict(title='28_dual_yaxis'), win='win28')

    #---29. Example for Latex Support
    win = viz.line(
        X=[1, 2, 3, 4],
        Y=[1, 4, 9, 16],
        name=r'$\alpha_{1c} = 352 \pm 11 \text{ km s}^{-1}$',
        update='append',
        opts={
            'showlegend': True,
            'title': "29_Latex",
            'xlabel': r'$\sqrt{(n_\text{c}(t|{T_\text{early}}))}$',
            'ylabel': r'$d, r \text{ (solar radius)}$',
        },
        win='win29'
    )
    viz.line(
        X=[1, 2, 3, 4],
        Y=[0.5, 2, 4.5, 8],
        win=win,
        name=r'$\beta_{1c} = 25 \pm 11 \text{ km s}^{-1}$',
        update='append',
    )

    #--- 30. SVG plotting
    svgstr = """
    <svg height="300" width="300">
      <ellipse cx="80" cy="80" rx="50" ry="30"
       style="fill:red;stroke:purple;stroke-width:2" />
      Sorry, your browser does not support inline SVG.
    </svg>
    """
    viz.svg(
        svgstr=svgstr,
        opts=dict(title='30_SVG_Rendering'),
        win='win_30'
    )

    #--- 31. close text window: 关闭窗口
    # viz.close(win=textwindow)
    # assert not viz.win_exists(textwindow), 'Closed window still exists'

    #--- 32. Arbitrary visdom content
    trace = dict(x=[1, 2, 3], y=[4, 5, 6], mode="markers+lines", type='custom',
                 marker={'color': 'red', 'symbol': 104, 'size': "10"},
                 text=["one", "two", "three"], name='1st Trace')
    layout = dict(title="32_Arbitrary_visdom_content", xaxis={'title': 'x1'},
                  yaxis={'title': 'x2'})

    viz._send({'data': [trace], 'layout': layout, 'win': 'win32'})

    #--- 33. PyTorch tensor
    try:
        import torch
        viz.line(Y=torch.Tensor([[0., 0.], [1., 1.]]), opts=dict(title='33_pytorch_tensor'), win='win33')
    except ImportError:
        print('Skipped PyTorch example')

    #--- 34. get/set state 设置和获取变量
    import json
    window = viz.text('test one', opts=dict(title='34_get_set_state'), win='win34')
    data = json.loads(viz.get_window_data())
    data[window]['content'] = 'test two'  # 获取指定window的状态
    viz.set_window_data(json.dumps(data))  # 更新状态

    #--- 35. audio demo:
    # tensor = np.random.uniform(-1, 1, 441000)
    # viz.audio(tensor=tensor, opts={'sample_frequency': 441000})  # 随机噪声
    # # download from http://www.externalharddrive.com/waves/animal/dolphin.wav
    # try:
    #     audio_url = 'http://www.externalharddrive.com/waves/animal/dolphin.wav'
    #     audiofile = os.path.join(tempfile.gettempdir(), 'dolphin.wav')
    #     urllib.request.urlretrieve(audio_url, audiofile)

    #     if os.path.isfile(audiofile):
    #         viz.audio(audiofile=audiofile)  # 下载音频
    # except BaseException:
    #     print('Skipped audio example')

    #--- 36. video demo:
    # try:
    #     video = np.empty([256, 250, 250, 3], dtype=np.uint8)
    #     for n in range(256):
    #         video[n, :, :, :].fill(n)
    #     viz.video(tensor=video)
    # except BaseException:
    #     print('Skipped video tensor example')

    # try:
    #     # video demo:
    #     # download video from http://media.w3.org/2010/05/sintel/trailer.ogv
    #     video_url = 'http://media.w3.org/2010/05/sintel/trailer.ogv'
    #     videofile = os.path.join(tempfile.gettempdir(), 'trailer.ogv')
    #     urllib.request.urlretrieve(video_url, videofile)

    #     if os.path.isfile(videofile):
    #         viz.video(videofile=videofile, opts={'width': 864, 'height': 480})
    # except BaseException as e:
    #     print('Skipped video file example', e)

    input('Waiting for callbacks, press enter to quit.')  # 阻塞程序, 按任意键结束

if __name__ == '__main__':
    DEFAULT_PORT = 8097
    DEFAULT_HOSTNAME = "http://localhost"
    parser = argparse.ArgumentParser(description='Demo arguments')
    parser.add_argument('-env', metavar='env', type=str, default='demo',
                        help='port the visdom server is running on.')
    parser.add_argument('-port', metavar='port', type=int, default=DEFAULT_PORT,
                        help='port the visdom server is running on.')
    parser.add_argument('-server', metavar='server', type=str,
                        default=DEFAULT_HOSTNAME,
                        help='Server address of the target to run the demo on.')
    parser.add_argument('-base_url', metavar='base_url', type=str,
                    default='/',
                    help='Base Url.')
    parser.add_argument('-use_incoming_socket', metavar='use_incoming_socket', type=bool,
                    default=True,
                    help='use_incoming_socket.')
    FLAGS = parser.parse_args()

    try:
        viz = Visdom(env='demo', port=FLAGS.port, server=FLAGS.server, base_url=FLAGS.base_url,
                use_incoming_socket=FLAGS.use_incoming_socket)
        run_demo(viz)
    except Exception as e:
        print("The visdom experienced an exception while running: {}\n".format(repr(e)))