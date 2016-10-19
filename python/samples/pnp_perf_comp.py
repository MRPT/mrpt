#!/usr/bin/env python
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import timeit

import mpld3
from mpld3 import plugins, utils

# Install tabulate using "pip-install tabulate"
from tabulate import tabulate

import os


def clear(): os.system('clear')
# Function to clear screen
import pymrpt


# Matplotlib figure parameters
mpl.rcParams['figure.figsize'] = (12.0, 8.0)
plt.rcParams.update({'axes.titlesize': 30, 'axes.labelsize': 20, 'xtick.labelsize': 15, 'ytick.labelsize': 15,
                     'figure.titlesize': 40})

# Define number of points and camera parameters
n = 10
sigma = 0.0005
n_range = range(5, 25)
sigma_range = np.arange(0.0001, 0.001, 0.0001)
f = 1.0
cx = 0.0
cy = 0.0
cam_intrinsic = np.array([[f, 0.0, cx], [0.0, f, cy], [0.0, 0.0, 1.0]])

# Define constants for serial outputting results
checkmark = u'\u2713'
l_progress_bar = 50

# Instantiate pnp module
pnp = pymrpt.pnp(n)

# Define settings for comparison module
algos = [pnp.dls, pnp.epnp, pnp.p3p,
         pnp.rpnp, pnp.ppnp, pnp.posit, pnp.lhm]
algo_names = ['dls', 'epnp', 'p3p', 'rpnp', 'ppnp', 'posit', 'lhm']
algo_ls = [':', '-', '--', '-', '--', '-', '-']
n_algos = len(algos)
n_iter = 100


class HighlightLines(plugins.PluginBase):
    # css format for interactive d3 plots
    """A plugin for an interactive legend.
    Inspired by http://bl.ocks.org/simzou/6439398
    """

    JAVASCRIPT = """
    mpld3.register_plugin("interactive_legend", InteractiveLegend);
    InteractiveLegend.prototype = Object.create(mpld3.Plugin.prototype);
    InteractiveLegend.prototype.constructor = InteractiveLegend;
    InteractiveLegend.prototype.requiredProps = ["line_ids", "labels"];
    InteractiveLegend.prototype.defaultProps = {}
    function InteractiveLegend(fig, props){
        mpld3.Plugin.call(this, fig, props);
    };
    InteractiveLegend.prototype.draw = function(){
        var labels = new Array();
        for(var i=0; i<this.props.labels.length; i++){
            var obj = {}
            obj.label = this.props.labels[i]
            obj.line = mpld3.get_element(this.props.line_ids[i], this.fig)
            obj.visible = false;
            labels.push(obj);
        }
        var ax = this.fig.axes[0]
        var legend = this.fig.canvas.append("svg:g")
                               .attr("class", "legend");
        // add the rectangles
        legend.selectAll("rect")
                .data(labels)
             .enter().append("rect")
                .attr("height",10)
                .attr("width", 25)
                .attr("x",ax.width+10+ax.position[0])
                .attr("y",function(d,i) {
                                return ax.position[1]+ i * 25 - 10;})
                .attr("stroke", function(d) {
                                return d.line.props.edgecolor})
                .attr("class", "legend-box")
                .style("fill", "white")
                .on("click", click)
        // add the text
        legend.selectAll("text")
                .data(labels)
            .enter().append("text")
              .attr("x", function (d) {
                return ax.width+10+ax.position[0] + 25 + 15
              })
              .attr("y", function(d,i) {
                return ax.position[1]+ i * 25
              })
              .text(function(d) { return d.label })
        // specify the action on click
        function click(d,i){
            d.visible = !d.visible;
            d3.select(this)
              .style("fill",function(d, i) {
                  console.log(d)
                  var color = d.line.props.edgecolor
                  return d.visible ? color : "white";
              })
            d3.select(d.line.path[0][0])
                .style("stroke-opacity", d.visible ? 1 : d.line.props.alpha);
        }
    };
    """

    def __init__(self, lines, labels, css):

        self.css_ = css or ""

        self.lines = lines
        self.dict_ = {"type": "interactive_legend",
                      "line_ids": [utils.get_id(line) for line in lines],
                      "labels": labels}


css = """
.legend-box {
  cursor: pointer;
}
"""


def vector2RotMat(vec, theta=0):
    # Function to convert from axis, angle to rotation matrix
    # Rodrigues rotation formula
    n_check = np.linalg.norm(vec)

    kx = vec[0] / n_check
    ky = vec[1] / n_check
    kz = vec[2] / n_check

    K = np.matrix([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
    I = np.identity(3)

    R = I + K * np.sin(theta) + K * K * (1 - np.cos(theta))
    R = np.array(R)

    return R


def quatvec2RotMat(q):
    # Function to convert from quaternion to Rotaiton matrix
    qw = np.sqrt(1 - np.linalg.norm(q) * np.linalg.norm(q))
    qx = q[0]
    qy = q[1]
    qz = q[2]

    R = [1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw,
         2 * qx * qy + 2 * qz * qw, 1 - 2 * qx * qx - 2 * qz * qz, 2 * qy * qz - 2 * qx * qw,
         2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx * qx - 2 * qy * qy]

    R = np.reshape(R, [3, 3])

    return R


def RotMat2quat(R):
    # Function to convert from rotation matrix to Quaternion
    qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2

    if qw > 0.01:
        qx = (R[2, 1] - R[1, 2]) / 4 / qw
        qy = (R[0, 2] - R[2, 0]) / 4 / qw
        qz = (R[1, 0] - R[0, 1]) / 4 / qw
    else:
        l = np.array([R[0, 0], R[1, 1], R[2, 2]])
        ind_max = np.argmax(l)

        if ind_max == 0:
            qx = np.sqrt((R[0, 0] + 1) / 2)
            qy = (R[1, 0] + R[0, 1]) / 4 / qx
            qz = (R[0, 2] + R[2, 0]) / 4 / qx

        elif ind_max == 1:
            qy = np.sqrt((R[1, 1] + 1) / 2)
            qx = (R[1, 0] + R[0, 1]) / 4 / qy
            qz = (R[2, 1] + R[1, 2]) / 4 / qy

        else:
            qz = np.sqrt((R[2, 2] + 1) / 2)
            qx = (R[0, 2] + R[2, 0]) / 4 / qz
            qy = (R[2, 1] + R[1, 2]) / 4 / qz

        qw = np.sqrt(1 - qx * qx - qy * qy - qz * qz)

    return [qw, qx, qy, qz]


def display_comparison_plot(t, arr, names, line_styles, title, xtitle, ytitle, ylim, figname):

    f, ax = plt.subplots()
    lines = []
    for i in np.arange(0, len(names)):
        l, = ax.plot(t, arr[:, i], label=names[i], lw=3, ls=line_styles[i])
        lines.append(l)

    leg = ax.legend(fancybox=True, shadow=True)
    leg.get_frame().set_alpha(0.8)

    lined = dict()
    for legline, origline in zip(leg.get_lines(), lines):
        legline.set_picker(10)
        lined[legline] = origline

    ax.set_xlabel(xtitle)
    ax.set_ylabel(ytitle)
    ax.set_title(title)
    ax = plt.gca()
    ax.set_ylim(ylim)
    ax.grid()

    def onpick(event):
        legline = event.artist
        origline = lined[legline]
        vis = not origline.get_visible()
        origline.set_visible(vis)

        if vis:
            legline.set_alpha(1)
        else:
            legline.set_alpha(0.2)

        f.canvas.draw()

    f.canvas.mpl_connect('pick_event', onpick)

    plt.show()
    plt.savefig(figname + '.pdf')


def display_comparison_plot_mpld3(t, arr, names, line_styles, title, xtitle, ytitle, ylim, figname):
    # Function used to generate interactive d3 plots in html
    f, ax = plt.subplots()
    lines = []
    for i in np.arange(0, len(names)):
        l, = ax.plot(t, arr[:, i], label=names[i], lw=3, ls=line_styles[i], alpha=0.2)
        lines.append(l)

    ax.set_xlabel(xtitle)
    ax.set_ylabel(ytitle)
    ax.set_title(title)
    ax = plt.gca()
    ax.set_ylim(ylim)
    ax.grid()

    plugins.connect(f, HighlightLines(lines, names, css))

    mpld3.display()

    #mpld3.save_html(f, figname + '.html')

    return mpld3.fig_to_html(f)


def printProgress(iteration, total, prefix='', suffix='', decimals=1, barLength=100):
    """
    # Print iterations progress
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        barLength   - Optional  : character length of bar (Int)
    """
    formatStr = "{0:." + str(decimals) + "f}"
    percents = formatStr.format(100 * (iteration / float(total)))
    filledLength = int(round(barLength * iteration / float(total)))
    bar = u"\u2588" * filledLength + '-' * (barLength - filledLength)
    sys.stdout.write('\r%s |%s| %s%s %s' % (prefix, bar, percents, '%', suffix)),
    sys.stdout.flush()
    if iteration == total:
        sys.stdout.write('\n')
        sys.stdout.flush()


def printTestStatus(test_index):
    # Print overall progress of all the tests
    head_tests = ['#No.', 'Status', 'Description']

    table_tests = [[str(1), '', 'Test for 100 iterations with each algorithm'], [str(2), '',  'Varitaion of error with image pixel noise standard deviation (' +
                                                                                 u'\u03C3' + ')'], [str(3), '', ' Variation in error with number of 2d/3d correspondences (n)'], [str(4), '', 'Average computation time of each algorithm']]

    for i in range(0, test_index):
        table_tests[i][1] = checkmark

    print tabulate(table_tests, head_tests, tablefmt='fancy_grid')

    for i in range(0, test_index):
        printProgress(l_progress_bar, l_progress_bar, prefix='Test' + str(i + 1) +
                      ' Progress:', suffix='Complete', barLength=50)


def printTest1Results(vals):
    # Function to print results of test 1
    test1_headers = ['Algo', 'Translation Mean Error', 'Translation Median Error',
                     'Rotation Mean Error', 'Rotation Median Error']

    test1_table = np.empty([7, 5], dtype=object)
    test1_table[:, 1:] = vals
    test1_table[:, 0] = algo_names

    print tabulate(test1_table, test1_headers, tablefmt='fancy_grid')


def printTest4Results(vals):
    # Function to print results of test 4
    test4_headers = ['Algo', 'Translation Mean Error', 'Translation Median Error',
                     'Rotation Mean Error', 'Rotation Median Error']

    vals = np.random.rand(7, 1)
    test4_table = np.empty([7, 2], dtype=object)
    test4_table[:, 1:] = vals
    test4_table[:, 0] = algo_names

    print tabulate(test4_table, test4_headers, tablefmt='fancy_grid')


def calc_err(pose1, pose2):
    # FUnction to compute reprojection errors
    if np.any(np.isnan(pose1)) or np.linalg.norm(pose1) > 1000000:
        err = [0, 0]
        return err

    # Percent error in translation
    t_est = np.array(pose1[0:3])
    t = np.array(pose2[0:3])
    err_t = (np.linalg.norm(t_est - t) / np.linalg.norm(t)) * 100
    # Rotation error
    q_est = pose1[3:6, 0]
    q = pose2[3:6, 0]

    if np.linalg.norm(q) != 0 and np.linalg.norm(q_est):
        val = np.dot(q_est, q) / np.linalg.norm(q_est) / np.linalg.norm(q)
    else:
        val = 1

    if val > 1:
        val = 1
    elif val < -1:
        val = -1
    elif val == np.nan:
        val = 1

    err_q = np.max(np.abs(np.arccos(val))) * 180 / np.pi
    err_q = err_q if err_q < 180 - err_q else 180 - err_q

    err = [err_t, err_q]

    if np.isnan(err_t) or np.isnan(err_q):
        print 'pose_est=\n', pose2
        print 'pose=\n', pose1

    return err


def err_plot():
    # Test 1
    # Define object points and image points

    obj_pts = np.random.randint(-40, 40, [n, 3])
    obj_pts = obj_pts.astype(float)
    obj_pts[0, :] = [0, 0, 0]
    img_pts = np.empty([n, 3])
    img_pts[:, 2] = 1

    pose_est = np.empty([6, 1])
    pose_act = np.empty([6, 1])

    err_net_t = []
    err_net_q = []

    for it in np.arange(0, n_iter):

        # Generate random camera extrinsic matrix
        v = 2 * np.random.random([3]) - np.array([1, 1, 1])
        v = v / np.linalg.norm(v)

        #R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
        R = vector2RotMat(v, np.pi * 2 / 3)
        q = RotMat2quat(R)
        t = np.array([0.0, 0.0, 200.0])

        # Compute image points based on actual extrinsic matrix and add noise to measurements
        for i in range(0, n):
            pt = np.dot(R, obj_pts[i, :]) + t
            img_pts[i, 0:2] = np.array([pt[0] / pt[2], pt[1] / pt[2]])

        img_pts[:, 0:2] = img_pts[:, 0:2] + sigma * np.random.randn(n, 2)

        pose_act[0:3, 0] = t
        pose_act[3:6, 0] = q[1:4]

        # Use the c-library to compute the pose
        err_t = []
        err_q = []
        for i in np.arange(0, n_algos):
            algos[i](obj_pts, img_pts, n, cam_intrinsic, pose_est)
            e = calc_err(pose_est, pose_act)
            err_t.append(e[0])
            err_q.append(e[1])

            if np.isnan(e[0]) or np.isnan(e[1]):
                print 'algo=\n', algo_names[i]
                print 'err=\n', e

        err_net_t.append(err_t)
        err_net_q.append(err_q)
        printProgress(it, n_iter, prefix='Test' + str(1) +
                      ' Progress:', suffix='Complete', barLength=50)
    mean_err_t = np.mean(err_net_t, axis=0)
    mean_err_q = np.mean(err_net_q, axis=0)
    median_err_t = np.median(err_net_t, axis=0)
    median_err_q = np.median(err_net_q, axis=0)

    vals = np.empty([7, 4])
    vals[:, 0] = mean_err_t
    vals[:, 1] = median_err_t
    vals[:, 2] = mean_err_q
    vals[:, 3] = median_err_q
    """
    for i in np.arange(0, n_algos):
        print 'mean_err_t_' + algo_names[i] + '=', mean_err_t[i], 'median_err_t_' + algo_names[i] + '=', median_err_t[i]
        print 'mean_err_q_' + algo_names[i] + '=', mean_err_q[i], 'median_err_q_' + algo_names[i] + '=', median_err_q[i]
    """
    it = np.arange(0, n_iter)

    err_net_t = np.array(err_net_t)
    err_net_q = np.array(err_net_q)

    s = '<h2> Translation error and Rotation error for 100 iterations (R - Randomly varying, t - fixed) </h2>'

    s1 = display_comparison_plot_mpld3(it, err_net_t, names=algo_names, line_styles=algo_ls,
                                       title='% Translation Error Plot', xtitle='Iteration', ytitle='e_t', ylim=[0, 2], figname='err_t')
    s2 = display_comparison_plot_mpld3(it, err_net_q, names=algo_names, line_styles=algo_ls,
                                       title='Rotation Error Plot (deg)', xtitle='Iteration', ytitle='e_q', ylim=[0, 1], figname='err_q')

    s = s + '\n <table > \n <tr > \n <td > \n' + s1 + \
        '</td > \n <td> \n' + s2 + '</td> \n </tr> \n </table> \n'

    return s, vals

    """
    comp_arr = np.zeros([n_iter,2])
    for i in np.arange(0,n_iter):
        comp_arr[i,:] = 2*mean_err_p3p
    nconv_epnp = float(np.sum(err_epnp>comp_arr))/n_iter*100
    nconv_dls  = float(np.sum(err_dls>comp_arr))/n_iter*100
    nconv_ppnp  = float(np.sum(err_ppnp>comp_arr))/n_iter*100
    nconv_posit  = float(np.sum(err_posit>comp_arr))/n_iter*100
    nconv_lhm  = float(np.sum(err_lhm>comp_arr))/n_iter*100
    nconv_p3p  = float(np.sum(err_p3p>comp_arr))/n_iter*100
    plt.figure()
    xvals = ['epnp', 'dls', 'ppnp', 'posit','lhm', 'p3p']
    xvals_int = np.arange(0,n_algos)
    yvals = [nconv_epnp, nconv_dls, nconv_ppnp, nconv_posit, nconv_lhm, nconv_p3p]
    plt.bar(xvals_int, yvals, align='center')
    plt.xticks(xvals_int, xvals)
    plt.title('%Divergence')
    plt.show()
    plt.savefig('divergence.pdf')
    """


def err_statistics_fcn_n():
    # Test 2
    mean_err_t_net = []
    mean_err_q_net = []
    median_err_t_net = []
    median_err_q_net = []

    for n in n_range:
        # Define object points and image points
        obj_pts = np.random.randint(-40, 40, [n, 3])
        obj_pts = obj_pts.astype(float)
        obj_pts[0, :] = [0, 0, 0]
        img_pts = np.empty([n, 3])
        img_pts[:, 2] = 1

        pose_est = np.empty([6, 1])
        pose_act = np.empty([6, 1])

        err_net_t = []
        err_net_q = []

        for it in np.arange(0, n_iter):

            # Define camera extrinsic matrix
            v = 2 * np.random.random([3]) - np.array([1, 1, 1])
            v = v / np.linalg.norm(v)

            #R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
            R = vector2RotMat(v, np.pi * 2 / 3)
            q = RotMat2quat(R)
            t = np.array([0.0, 0.0, 200.0])

            # Compute image points based on actual extrinsic matrix and add noise to measurements
            for i in range(0, n):
                pt = np.dot(R, obj_pts[i, :]) + t
                img_pts[i, 0:2] = np.array([pt[0] / pt[2], pt[1] / pt[2]])

            img_pts[:, 0:2] = img_pts[:, 0:2] + sigma * np.random.randn(n, 2)

            pose_act[0:3, 0] = t
            pose_act[3:6, 0] = q[1:4]

            # Use the c-library to compute the pose

            err_t = []
            err_q = []
            for i in range(0, n_algos):
                algos[i](obj_pts, img_pts, n, cam_intrinsic, pose_est)
                e = calc_err(pose_est, pose_act)
                err_t.append(e[0])
                err_q.append(e[1])

            err_net_t.append(err_t)
            err_net_q.append(err_q)

        mean_err_t_net.append(np.mean(err_net_t, axis=0))
        mean_err_q_net.append(np.mean(err_net_q, axis=0))

        median_err_t_net.append(np.median(err_net_t, axis=0))
        median_err_q_net.append(np.median(err_net_q, axis=0))

        printProgress(n - 5, len(n_range), prefix='Test' + str(3) +
                      ' Progress:', suffix='Complete', barLength=50)

    it = np.arange(5, 25)

    mean_err_t_net = np.array(mean_err_t_net)
    mean_err_q_net = np.array(mean_err_q_net)
    median_err_t_net = np.array(median_err_t_net)
    median_err_q_net = np.array(median_err_q_net)

    s = '<h2> Mean and Median error in Translation and Rotation with varying 2d/3d correspondences (n) </h2>'

    s1 = display_comparison_plot_mpld3(it, mean_err_t_net, names=algo_names, line_styles=algo_ls,
                                       title='Mean Translation %Error Plot', xtitle='n', ytitle=r'% Translation error e_t', ylim=[0, 10], figname='mean_err_t')
    s2 = display_comparison_plot_mpld3(it, mean_err_q_net, names=algo_names, line_styles=algo_ls,
                                       title='Mean Rotation Error Plot (deg)', xtitle='n', ytitle=r'Rotation error e_q (deg)', ylim=[0, 0.5], figname='mean_err_q')

    s3 = display_comparison_plot_mpld3(it, median_err_t_net, names=algo_names, line_styles=algo_ls,
                                       title='Median Translation %Error Plot', xtitle='n', ytitle=r'% Translation error e_t', ylim=[0, 1], figname='median_err_t')
    s4 = display_comparison_plot_mpld3(it, median_err_q_net, names=algo_names, line_styles=algo_ls,
                                       title='Median Rotation Error Plot (deg)', xtitle='n', ytitle=r'Rotation error e_q(deg)', ylim=[0, 0.5], figname='median_err_q')

    s = s + '\n<table>\n <tr>\n <td>\n' + s1 + '</td>\n <td>\n' + s2 + '</td>\n </tr>\n' + \
        '\n<tr>\n <td>\n' + s3 + '\n</td>\n <td>\n' + s4 + '\n</td>\n </tr>\n </table>\n'
    return s


def err_statistics_fcn_sigma():
    # Test 3
    mean_err_t_net = []
    mean_err_q_net = []
    median_err_t_net = []
    median_err_q_net = []

    n = 10

    for sigma in sigma_range:
        # Define object points and image points
        obj_pts = np.random.randint(-40, 40, [n, 3])
        obj_pts = obj_pts.astype(float)
        obj_pts[0, :] = [0, 0, 0]
        img_pts = np.empty([n, 3])
        img_pts[:, 2] = 1

        pose_est = np.empty([6, 1])
        pose_act = np.empty([6, 1])

        err_net_t = []
        err_net_q = []

        for it in np.arange(0, n_iter):

            # Define camera extrinsic matrix
            v = 2 * np.random.random([3]) - np.array([1, 1, 1])
            v = v / np.linalg.norm(v)

            #R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
            R = vector2RotMat(v, np.pi * 2 / 3)
            q = RotMat2quat(R)
            t = np.array([0.0, 0.0, 200.0])

            # Compute image points based on actual extrinsic matrix and add noise to measurements
            for i in range(0, n):
                pt = np.dot(R, obj_pts[i, :]) + t
                img_pts[i, 0:2] = np.array([pt[0] / pt[2], pt[1] / pt[2]])

            img_pts[:, 0:2] = img_pts[:, 0:2] + sigma * np.random.randn(n, 2)

            pose_act[0:3, 0] = t
            pose_act[3:6, 0] = q[1:4]

            # Use the c-library to compute the pose
            err_t = []
            err_q = []
            for i in range(0, n_algos):
                algos[i](obj_pts, img_pts, n, cam_intrinsic, pose_est)
                e = calc_err(pose_est, pose_act)
                err_t.append(e[0])
                err_q.append(e[1])

            err_net_t.append(err_t)
            err_net_q.append(err_q)

        mean_err_t_net.append(np.mean(err_net_t, axis=0))
        mean_err_q_net.append(np.mean(err_net_q, axis=0))

        median_err_t_net.append(np.median(err_net_t, axis=0))
        median_err_q_net.append(np.median(err_net_q, axis=0))

        printProgress(sigma * 1000, len(sigma_range), prefix='Test' + str(2) +
                      ' Progress:', suffix='Complete', barLength=50)

    it = np.arange(0.001, 0.010, 0.001)

    mean_err_t_net = np.array(mean_err_t_net)
    mean_err_q_net = np.array(mean_err_t_net)

    median_err_t_net = np.array(median_err_t_net)
    median_err_q_net = np.array(median_err_t_net)

    s = '\n<h2>\n Mean and Median error in Translation and Rotation with varying noise variance (sigma)\n </h2>\n'

    s1 = display_comparison_plot_mpld3(it, mean_err_t_net, names=algo_names, line_styles=algo_ls, title='Mean Translation %Error Plot',
                                       xtitle=r'\sigma', ytitle=r'% Translation error e_t', ylim=[0, 10], figname='mean_sigma_err_t')
    s2 = display_comparison_plot_mpld3(it, mean_err_q_net, names=algo_names, line_styles=algo_ls, title='Mean Rotation Error Plot (deg)',
                                       xtitle=r'\sigma', ytitle=r'Rotation error e_q (deg)', ylim=[0, 0.5], figname='mean_sigma_err_q')

    s3 = display_comparison_plot_mpld3(it, median_err_t_net, names=algo_names, line_styles=algo_ls, title='Median Translation %Error Plot',
                                       xtitle=r'\sigma', ytitle=r'% Translation error e_t', ylim=[0, 1], figname='median_sigma_err_t')
    s4 = display_comparison_plot_mpld3(it, median_err_q_net, names=algo_names, line_styles=algo_ls, title='Median Rotation Error Plot (deg)',
                                       xtitle=r'\sigma', ytitle=r'Rotation error e_q (deg)', ylim=[0, 0.5], figname='median_sigma_err_q')

    s = s + '\n<table>\n <tr>\n <td>\n' + s1 + '\n</td>\n <td>\n' + s2 + '\n</td>\n </tr>\n' + \
        '\n<tr>\n <td>\n' + s3 + '\n</td> \n<td>\n' + s4 + '\n</td>\n </tr>\n </table>\n'

    return s


def time_comp():
    # Test 4
    obj_pts_store = []
    img_pts_store = []
    n_max = 50
    n_step = 1
    n_start = 10
    n_iter = 10

    tcomp_storage = []

    for n in np.arange(n_start, n_max, n_step):
        # Generate object points and image points
        for i in np.arange(0, n_iter):
            obj_pts = np.random.randint(-40, 40, [n, 3])
            obj_pts = obj_pts.astype(float)
            obj_pts[0, :] = [0, 0, 0]
            img_pts = np.empty([n, 3])
            img_pts[:, 2] = 1

            # Define camera extrinsic matrix
            v = 2 * np.random.random([3]) - np.array([1, 1, 1])
            v = v / np.linalg.norm(v)

            #R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
            R = vector2RotMat(v, np.pi * 2 / 3)
            t = np.array([0.0, 0.0, 200.0])

            # Compute image points based on actual extrinsic matrix
            for i in range(0, n):
                pt = np.dot(R, obj_pts[i, :]) + t
                img_pts[i, 0:2] = np.array([pt[0] / pt[2], pt[1] / pt[2]])

            # Add noise to measurements
            img_pts[:, 0:2] = img_pts[:, 0:2] + sigma * np.random.randn(n, 2)

            obj_pts_store.append(obj_pts)
            img_pts_store.append(img_pts)

        tcomp = []
        # Compute time for n_iter iterations
        for it in np.arange(1, n_algos):
            pose_est = np.empty([6, 1])
            start = timeit.default_timer()
            for i in np.arange(0, n_iter):
                obj_pts = obj_pts_store[i]
                img_pts = img_pts_store[i]
                algos[it](obj_pts, img_pts, n, cam_intrinsic, pose_est)

            end = timeit.default_timer()

            tcomp.append((end - start) / float(n_iter) * 1000.0)

        tcomp_storage.append(tcomp)

        printProgress(n - n_start, len(range(n_start, n_max, n_step)), prefix='Test' + str(4) +
                      ' Progress:', suffix='Complete', barLength=50)
    it = np.arange(n_start, n_max, n_step)
    tcomp_storage = np.array(tcomp_storage)

    s = '\n<h2>\n Average compuational time for algorithm (ms) \n</h2>\n'
    s = s + display_comparison_plot_mpld3(it, tcomp_storage, names=algo_names[1:], line_styles=algo_ls[
        1:], title='Average Time for algorithm (ms)', xtitle=r'n', ytitle=r't(ms)', ylim=[0, 0.6], figname='mean_time')

    return s

# Introduction and links to various files
ss = """<!DOCTYPE html>
<html>
<body bgcolor="#E6E6FA">
<br>
<CENTER>
<embed src = "https://www.dropbox.com/s/a266gqe3o0typpg/pnp_intro1.png?raw=1 #toolbar=0&navpanes=0&scrollbar=0" width = "1000" height = "1250" ALIGN=CENTER>
<br>
<embed src = "https://www.dropbox.com/s/5v8n4edc7g8q8tu/pnp_intro2.png?raw=1 #toolbar=0&navpanes=0&scrollbar=0" width = "1000" height = "1250" ALIGN=CENTER>
<br>
<embed src = "https://www.dropbox.com/s/6bdadtn99tthth0/pnp_intro3.png?raw=1 #toolbar=0&navpanes=0&scrollbar=0" width = "1000" height = "650" ALIGN=CENTER>
<br>
<h1> Sample Pose Estimation using Camera Calib application of MRPT </h1>
<br>
<iframe src = "https://www.youtube.com/embed/aGd7ZyrcwaE" width = "960" height = "540" frameborder = "0" allowfullscreen ALIGN=CENTER> </iframe >

<br>
<br>
<h1> Performance Comparison using python interface in MRPT(pnp_perf_comp.py) </h1>
<div>
<h3> Plots are interactive <br>
    * Click on legend box to solidy the particular algorithm <br>
    * Pan and Zoom at lower left corner  </h3>
</div>"""

# sys.stderr.write('\x1b[2J\x1b[H')
clear()
printTestStatus(0)
printProgress(0, len(range(5, 25)), prefix='Test' + str(1) +
              ' Progress:', suffix='Complete', barLength=50)
s1, vals = err_plot()

# sys.stderr.write('\x1b[2J\x1b[H')
clear()
printTestStatus(1)
printProgress(0, len(range(5, 25)), prefix='Test' + str(2) +
              ' Progress:', suffix='Complete', barLength=50)
s2 = err_statistics_fcn_sigma()

# sys.stderr.write('\x1b[2J\x1b[H')
clear()
printTestStatus(2)
printProgress(0, len(range(5, 25)), prefix='Test' + str(3) +
              ' Progress:', suffix='Complete', barLength=50)
s3 = err_statistics_fcn_n()

# sys.stderr.write('\x1b[2J\x1b[H')
clear()
printTestStatus(3)
printProgress(0, len(range(5, 25)), prefix='Test' + str(4) +
              ' Progress:', suffix='Complete', barLength=50)
s4 = time_comp()

# sys.stderr.write('\x1b[2J\x1b[H')
clear()
printTestStatus(4)
print '\n\nResults of Test 1 \n\n'
printTest1Results(vals)

s5 = """<h1> MRPT Merge Pull Request </h1>
  <h3>
  <a href="https://github.com/MRPT/mrpt/pull/310">Link to PnP Algorithm Pull Request </a>
  </h3>
</CENTER>
</body>
</html>
"""

ss = ss + s1 + s2 + s3 + s4 + s5

f = open("pnp_perf_comp.html", "w")
f.write(ss)
f.close()
