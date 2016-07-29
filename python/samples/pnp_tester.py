#!/usr/bin/env python
import sys
sys.path.append("../../build/lib")
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

import mpld3
from mpld3 import plugins, utils

class HighlightLines(plugins.PluginBase):
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
                      "labels":labels}


css = """
.legend-box {
  cursor: pointer;  
}
"""

import pymrpt

mpl.rcParams['figure.figsize'] = (20.0, 16.0)
plt.rcParams.update({'axes.titlesize': 40,'axes.labelsize': 25,'xtick.labelsize': 20, 'ytick.labelsize':15, 
                    'figure.titlesize':40})
n_algos=6                    

def vector2RotMat(vec, theta=0):
    #Rodrigues rotation formula
    n_check=np.linalg.norm(vec)
    
    kx=vec[0]/n_check
    ky=vec[1]/n_check
    kz=vec[2]/n_check
    
    K=np.matrix([[0, -kz, ky],[kz, 0, -kx],[-ky, kx, 0]])
    I=np.identity(3)
    
    R=I+K*np.sin(theta)+K*K*(1-np.cos(theta))
    R=np.array(R)
    
    return R
    
def quatvec2RotMat(q):
    qw = np.sqrt(1- np.linalg.norm(q)*np.linalg.norm(q))
    qx=q[0]
    qy=q[1]
    qz=q[2]

    R = [1-2*qy*qy-2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz+2*qy*qw,
         2*qx*qy + 2*qz*qw,1-2*qx*qx-2*qz*qz, 2*qy*qz - 2*qx*qw,
         2*qx*qz-2*qy*qw,2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]
         
    R=np.reshape(R, [3,3])
         
    return R
    
def RotMat2quat(R):
    qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2] )/2
    
    if qw > 0.01:
        qx = (R[2,1] - R[1,2])/4/qw
        qy = (R[0,2] - R[2,0])/4/qw
        qz = (R[1,0] - R[0,1])/4/qw
    else:
        l = np.array([ R[0,0], R[1,1], R[2,2] ])
        ind_max = np.argmax(l)
        
        if ind_max == 0:
            qx = np.sqrt((R[0,0]+1)/2)
            qy = (R[1,0] + R[0,1])/4/qx
            qz = (R[0,2] + R[2,0])/4/qx
            
        elif ind_max==1:
            qy = np.sqrt((R[1,1]+1)/2)
            qx = (R[1,0] + R[0,1])/4/qy
            qz = (R[2,1] + R[1,2])/4/qy
        
        else: 
            qz = np.sqrt((R[2,2]+1)/2)
            qx = (R[0,2] + R[2,0])/4/qz
            qy = (R[2,1] + R[1,2])/4/qz
            
        qw = np.sqrt(1- qx*qx -qy*qy - qz*qz)
        
    return [qw, qx, qy, qz]

def display_comparison_plot(t, arr, names, line_styles, title, xtitle, ytitle, ylim, figname):

    f, ax = plt.subplots()    
    lines=[]
    for i in np.arange(0,len(arr)):
        l, =ax.plot(t,arr[i,:],label=names[i], lw=2, ls=line_styles[i])
        lines.append(l)
    
    leg = ax.legend(fancybox=True, shadow=True)
    leg.get_frame().set_alpha(0.8)
    
    lined = dict()
    for legline, origline in zip(leg.get_lines(), lines):
        legline.set_picker(10)
        lined[legline]=origline
        
    ax.set_xlabel(xtitle)
    ax.set_ylabel(ytitle)
    ax.set_title(title)
    ax=plt.gca()
    ax.set_ylim(ylim)
    ax.grid()
    
    def onpick(event):
        legline=event.artist
        origline = lined[legline]
        vis=not origline.get_visible()
        origline.set_visible(vis)
        
        if vis:
            legline.set_alpha(1)
        else:
            legline.set_alpha(0.2)
        
        f.canvas.draw()
    
    f.canvas.mpl_connect('pick_event', onpick)
        
    plt.show()
    plt.savefig(figname+'.pdf')
 
    
def display_comparison_plot_mpld3(t, arr, names, line_styles, title, xtitle, ytitle, ylim, figname):
    f, ax = plt.subplots()    
    lines=[]
    for i in np.arange(0,len(arr)):
        l, =ax.plot(t,arr[i,:],label=names[i], lw=3, ls=line_styles[i], alpha=0.2)
        lines.append(l)
    
    ax.set_xlabel(xtitle)
    ax.set_ylabel(ytitle)
    ax.set_title(title)
    ax=plt.gca()
    ax.set_ylim(ylim)
    ax.grid()
    
    plugins.connect(f, HighlightLines(lines, names, css))

    mpld3.display()
    
    mpld3.save_html(f, figname+'.html')

def calc_err(pose1, pose2):
    # Percent error in translation    
    t_est = np.array(pose1[0:3])
    t     = np.array(pose2[0:3])
    err_t = (np.linalg.norm(t_est-t)/np.linalg.norm(t))*100
    # Rotation error
    q_est = pose1[3:6,0]
    q     = pose2[3:6,0]
    err_q = np.max(np.abs(np.arccos(np.dot(q_est,q)/np.linalg.norm(q_est)/np.linalg.norm(q))))
    err_q = err_q if err_q < np.pi-err_q else np.pi-err_q   
    err = [err_t, err_q]
    return err

# Define number of points and camera parameters
n=10
sigma=0.005
f=1.0
cx=0.0
cy=0.0

# Instantiate pnp module
pnp = pymrpt.pnp(n)

def err_plot():
    # Define object points and image points
    
    obj_pts=np.random.randint(-40,40,[n,3])
    obj_pts=obj_pts.astype(float)
    obj_pts[0,:]=[0,0,0]
    img_pts=np.empty([n,3])
    img_pts[:,2]=1
    
    pose_epnp=np.empty([6,1])
    pose_dls=np.empty([6,1])
    pose_p3p=np.empty([6,1])
    pose_ppnp=np.empty([6,1])
    pose_posit=np.empty([6,1])
    pose_lhm=np.empty([6,1])
    pose_act=np.empty([6,1])
    
    n_iter=100
    
    err_epnp=[]
    err_dls=[]
    err_p3p=[]
    err_ppnp=[]
    err_posit=[]
    err_lhm=[]
    
    for it in np.arange(0,n_iter):
    
        # Define camera extrinsic matrix
        v=2*np.random.random([3]) - np.array([1,1,1])
        v=v/np.linalg.norm(v)
        
        #R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
        R = vector2RotMat(v, np.pi*2/3)
        q = RotMat2quat(R)
        t=np.array([0.0,0.0,200.0])
        
        # Compute image points based on actual extrinsic matrix and add noise to measurements
        for i in range(0,n):
            pt=np.dot(R,obj_pts[i,:])+t
            img_pts[i,0:2]= np.array([pt[0]/pt[2] , pt[1]/pt[2]])
        
        img_pts[:,0:2]=img_pts[:,0:2] + sigma*np.random.randn(n,2)
        
        
        pose_act[0:3,0]=t;
        pose_act[3:6,0]=q[1:4];
        cam_intrinsic=np.array([[f,0.0,cx],[0.0,f,cy],[0.0, 0.0, 1.0]])
        
        # Use the c-library to compute the pose 
        pnp.epnp_solve(obj_pts,img_pts, n, cam_intrinsic, pose_epnp)
        e=calc_err(pose_epnp, pose_act)
        err_epnp.append(e)
        
        pnp.dls_solve(obj_pts, img_pts, n, cam_intrinsic, pose_dls)
        e=calc_err(pose_dls, pose_act)
        err_dls.append(e)
        
        pnp.p3p_solve(obj_pts, img_pts, n, cam_intrinsic, pose_p3p)
        e=calc_err(pose_p3p, pose_act)
        err_p3p.append(e)
        
        pnp.ppnp_solve(obj_pts,img_pts, n, cam_intrinsic, pose_ppnp)
        e=calc_err(pose_ppnp, pose_act)
        err_ppnp.append(e)
        
        pnp.posit_solve(obj_pts,img_pts,n, cam_intrinsic, pose_posit)
        e=calc_err(pose_posit, pose_act)
        err_posit.append(e)
        
        pnp.lhm_solve(obj_pts,img_pts,n,cam_intrinsic,pose_lhm)
        e=calc_err(pose_lhm, pose_act)
        err_lhm.append(e)
        
        #pnp.upnp_solve(obj_pts,img_pts, n, cam_intrinsic, pose_upnp)
        #e=calc_err(pose_upnp, pose_act)
        #err_upnp.append(e)
    
    mean_err_epnp = np.mean(err_epnp, axis=0)
    mean_err_dls = np.mean(err_dls, axis=0)
    mean_err_ppnp = np.mean(err_ppnp, axis=0)
    mean_err_posit = np.mean(err_posit, axis=0)
    mean_err_lhm = np.mean(err_lhm, axis=0)
    mean_err_p3p = np.mean(err_p3p, axis=0)
    
    median_err_epnp = np.median(err_epnp, axis=0)
    median_err_dls = np.median(err_dls, axis=0)
    median_err_ppnp = np.median(err_ppnp, axis=0)
    median_err_posit = np.median(err_posit, axis=0)
    median_err_lhm = np.median(err_lhm, axis=0)
    median_err_p3p = np.median(err_p3p, axis=0)
    
    
    print 'e_bar_epnp=', mean_err_epnp, 'e_median_epnp=', median_err_epnp
    print 'e_bar_dls=', mean_err_dls, 'e_median_dls=', median_err_dls
    print 'e_bar_ppnp=', mean_err_ppnp, 'e_median_ppnp=', median_err_ppnp
    print 'e_bar_posit=', mean_err_posit, 'e_median_posit=', median_err_posit
    print 'e_bar_lhm=', mean_err_lhm, 'e_median_lhm=', median_err_lhm
    print 'e_bar_p3p=', mean_err_p3p, 'e_median_p3p=', median_err_p3p
    
        
    
    err=np.array(err_epnp + err_dls + err_p3p + err_ppnp + err_posit + err_lhm)
    
    err_t=err[:,0].reshape(n_algos,n_iter)
    err_q=err[:,1].reshape(n_algos,n_iter)
    
    
    it=np.arange(0,n_iter)
    
    
    display_comparison_plot_mpld3(it, err_t, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Translation %Error Plot', xtitle='Iteration', ytitle='%$e_t$', ylim=[0,10], figname ='err_t')
    
    display_comparison_plot_mpld3(it, err_q, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Rotation Error Plot (rad)', xtitle='Iteration', ytitle='$e_q$', ylim=[0,2], figname ='err_q')
    
    
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


def err_statistics_fcn_n():
    
    mean_epnp= []
    mean_dls = []
    mean_ppnp= []
    mean_posit=[]
    mean_lhm = []
    mean_p3p = []
    
    median_epnp= []
    median_dls = []
    median_ppnp= []
    median_posit=[]
    median_lhm = []
    median_p3p = []
    
    for n in np.arange(5,25):
        #Define object points and image points
        obj_pts=np.random.randint(-40,40,[n,3])
        obj_pts=obj_pts.astype(float)
        obj_pts[0,:]=[0,0,0]
        img_pts=np.empty([n,3])
        img_pts[:,2]=1
        
        pose_epnp=np.empty([6,1])
        pose_dls=np.empty([6,1])
        pose_p3p=np.empty([6,1])
        pose_ppnp=np.empty([6,1])
        pose_posit=np.empty([6,1])
        pose_lhm=np.empty([6,1])
        pose_act=np.empty([6,1])
        
        n_iter=100
        
        err_epnp=[]
        err_dls=[]
        err_p3p=[]
        err_ppnp=[]
        err_posit=[]
        err_lhm=[]
        
        for it in np.arange(0,n_iter):
        
            # Define camera extrinsic matrix
            v=2*np.random.random([3]) - np.array([1,1,1])
            v=v/np.linalg.norm(v)
            
            #R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
            R = vector2RotMat(v, np.pi*2/3)
            q = RotMat2quat(R)
            t=np.array([0.0,0.0,200.0])
            
            # Compute image points based on actual extrinsic matrix and add noise to measurements
            for i in range(0,n):
                pt=np.dot(R,obj_pts[i,:])+t
                img_pts[i,0:2]= np.array([pt[0]/pt[2] , pt[1]/pt[2]])
            
            img_pts[:,0:2]=img_pts[:,0:2] + sigma*np.random.randn(n,2)
            
            pose_act[0:3,0]=t;
            pose_act[3:6,0]=q[1:4];
            cam_intrinsic=np.array([[f,0.0,cx],[0.0,f,cy],[0.0, 0.0, 1.0]])
            
            # Use the c-library to compute the pose 
            pnp.epnp_solve(obj_pts,img_pts, n, cam_intrinsic, pose_epnp)
            e=calc_err(pose_epnp, pose_act)
            err_epnp.append(e)
            
            pnp.dls_solve(obj_pts, img_pts, n, cam_intrinsic, pose_dls)
            e=calc_err(pose_dls, pose_act)
            err_dls.append(e)
            
            pnp.p3p_solve(obj_pts, img_pts, n, cam_intrinsic, pose_p3p)
            e=calc_err(pose_p3p, pose_act)
            err_p3p.append(e)
            
            pnp.ppnp_solve(obj_pts,img_pts, n, cam_intrinsic, pose_ppnp)
            e=calc_err(pose_ppnp, pose_act)
            err_ppnp.append(e)
            
            pnp.posit_solve(obj_pts,img_pts,n, cam_intrinsic, pose_posit)
            e=calc_err(pose_posit, pose_act)
            err_posit.append(e)
            
            pnp.lhm_solve(obj_pts,img_pts,n,cam_intrinsic,pose_lhm)
            e=calc_err(pose_lhm, pose_act)
            err_lhm.append(e)
        
        mean_err_epnp = np.mean(err_epnp, axis=0)
        mean_err_dls = np.mean(err_dls, axis=0)
        mean_err_ppnp = np.mean(err_ppnp, axis=0)
        mean_err_posit = np.mean(err_posit, axis=0)
        mean_err_lhm = np.mean(err_lhm, axis=0)
        mean_err_p3p = np.mean(err_p3p, axis=0)
        
        median_err_epnp = np.median(err_epnp, axis=0)
        median_err_dls = np.median(err_dls, axis=0)
        median_err_ppnp = np.median(err_ppnp, axis=0)
        median_err_posit = np.median(err_posit, axis=0)
        median_err_lhm = np.median(err_lhm, axis=0)
        median_err_p3p = np.median(err_p3p, axis=0)
        
        mean_epnp.append(mean_err_epnp)
        mean_dls.append(mean_err_dls)
        mean_ppnp.append(mean_err_ppnp)
        mean_posit.append(mean_err_posit)
        mean_lhm.append(mean_err_lhm)
        mean_p3p.append(mean_err_p3p)

        median_epnp.append(median_err_epnp)
        median_dls.append(median_err_dls)
        median_ppnp.append(median_err_ppnp)
        median_posit.append(median_err_posit)
        median_lhm.append(median_err_lhm)
        median_p3p.append(median_err_p3p)
        
    it =np.arange(5,25)

    err=np.array(mean_epnp + mean_dls + mean_p3p + mean_ppnp + mean_posit + mean_lhm)

    n_iter1=len(it)

    err_t=err[:,0].reshape(n_algos,n_iter1)
    err_q=err[:,1].reshape(n_algos,n_iter1)        
    
    display_comparison_plot_mpld3(it, err_t, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Mean Translation %Error Plot', xtitle='n', ytitle=r'% $\bar{e}_t$', ylim=[0,10], figname ='mean_err_t')
    display_comparison_plot_mpld3(it, err_q, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Mean Rotation Error Plot (rad)', xtitle='n', ytitle=r'$\bar{e}_q$', ylim=[0,0.5], figname ='mean_err_q')

    display_comparison_plot_mpld3(it, err_t, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Median Translation %Error Plot', xtitle='n', ytitle=r'% $\tilde{e}_t$', ylim=[0,10], figname ='median_err_t')
    display_comparison_plot_mpld3(it, err_q, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Median Rotation Error Plot (rad)', xtitle='n', ytitle=r'$\tilde{e}_q$', ylim=[0,0.5], figname ='median_err_q')


    return err    
    
def err_statistics_fcn_sigma():
    
    mean_epnp= []
    mean_dls = []
    mean_ppnp= []
    mean_posit=[]
    mean_lhm = []
    mean_p3p = []
    
    median_epnp= []
    median_dls = []
    median_ppnp= []
    median_posit=[]
    median_lhm = []
    median_p3p = []
    
    n=10
    
    for sigma in np.arange(0.001,0.010,0.001):
        #Define object points and image points
        obj_pts=np.random.randint(-40,40,[n,3])
        obj_pts=obj_pts.astype(float)
        obj_pts[0,:]=[0,0,0]
        img_pts=np.empty([n,3])
        img_pts[:,2]=1
        
        pose_epnp=np.empty([6,1])
        pose_dls=np.empty([6,1])
        pose_p3p=np.empty([6,1])
        pose_ppnp=np.empty([6,1])
        pose_posit=np.empty([6,1])
        pose_lhm=np.empty([6,1])
        pose_act=np.empty([6,1])
        
        n_iter=100
        
        err_epnp=[]
        err_dls=[]
        err_p3p=[]
        err_ppnp=[]
        err_posit=[]
        err_lhm=[]
        
        for it in np.arange(0,n_iter):
        
            # Define camera extrinsic matrix
            v=2*np.random.random([3]) - np.array([1,1,1])
            v=v/np.linalg.norm(v)
            
            #R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
            R = vector2RotMat(v, np.pi*2/3)
            q = RotMat2quat(R)
            t=np.array([0.0,0.0,200.0])
            
            # Compute image points based on actual extrinsic matrix and add noise to measurements
            for i in range(0,n):
                pt=np.dot(R,obj_pts[i,:])+t
                img_pts[i,0:2]= np.array([pt[0]/pt[2] , pt[1]/pt[2]])
            
            img_pts[:,0:2]=img_pts[:,0:2] + sigma*np.random.randn(n,2)
            
            pose_act[0:3,0]=t;
            pose_act[3:6,0]=q[1:4];
            cam_intrinsic=np.array([[f,0.0,cx],[0.0,f,cy],[0.0, 0.0, 1.0]])
            
            # Use the c-library to compute the pose 
            pnp.epnp_solve(obj_pts,img_pts, n, cam_intrinsic, pose_epnp)
            e=calc_err(pose_epnp, pose_act)
            err_epnp.append(e)
            
            pnp.dls_solve(obj_pts, img_pts, n, cam_intrinsic, pose_dls)
            e=calc_err(pose_dls, pose_act)
            err_dls.append(e)
            
            pnp.p3p_solve(obj_pts, img_pts, n, cam_intrinsic, pose_p3p)
            e=calc_err(pose_p3p, pose_act)
            err_p3p.append(e)
            
            pnp.ppnp_solve(obj_pts,img_pts, n, cam_intrinsic, pose_ppnp)
            e=calc_err(pose_ppnp, pose_act)
            err_ppnp.append(e)
            
            pnp.posit_solve(obj_pts,img_pts,n, cam_intrinsic, pose_posit)
            e=calc_err(pose_posit, pose_act)
            err_posit.append(e)
            
            pnp.lhm_solve(obj_pts,img_pts,n,cam_intrinsic,pose_lhm)
            e=calc_err(pose_lhm, pose_act)
            err_lhm.append(e)
        
        mean_err_epnp = np.mean(err_epnp, axis=0)
        mean_err_dls = np.mean(err_dls, axis=0)
        mean_err_ppnp = np.mean(err_ppnp, axis=0)
        mean_err_posit = np.mean(err_posit, axis=0)
        mean_err_lhm = np.mean(err_lhm, axis=0)
        mean_err_p3p = np.mean(err_p3p, axis=0)
        
        median_err_epnp = np.median(err_epnp, axis=0)
        median_err_dls = np.median(err_dls, axis=0)
        median_err_ppnp = np.median(err_ppnp, axis=0)
        median_err_posit = np.median(err_posit, axis=0)
        median_err_lhm = np.median(err_lhm, axis=0)
        median_err_p3p = np.median(err_p3p, axis=0)
        
        mean_epnp.append(mean_err_epnp)
        mean_dls.append(mean_err_dls)
        mean_ppnp.append(mean_err_ppnp)
        mean_posit.append(mean_err_posit)
        mean_lhm.append(mean_err_lhm)
        mean_p3p.append(mean_err_p3p)

        median_epnp.append(median_err_epnp)
        median_dls.append(median_err_dls)
        median_ppnp.append(median_err_ppnp)
        median_posit.append(median_err_posit)
        median_lhm.append(median_err_lhm)
        median_p3p.append(median_err_p3p)
        
    it =np.arange(0.001,0.010,0.001)

    err=np.array(mean_epnp + mean_dls + mean_p3p + mean_ppnp + mean_posit + mean_lhm)

    n_iter1=len(it)

    err_t=err[:,0].reshape(n_algos,n_iter1)
    err_q=err[:,1].reshape(n_algos,n_iter1)        
    
    display_comparison_plot_mpld3(it, err_t, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Mean Translation %Error Plot', xtitle=r'\sigma', ytitle=r'% $\bar{e}_t$', ylim=[0,10], figname ='mean_sigma_err_t')
    display_comparison_plot_mpld3(it, err_q, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Mean Rotation Error Plot (rad)', xtitle=r'\sigma', ytitle=r'$\bar{e}_q$', ylim=[0,0.5], figname ='mean_sigma_err_q')

    display_comparison_plot_mpld3(it, err_t, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Median Translation %Error Plot', xtitle=r'\sigma', ytitle=r'% $\tilde{e}_t$', ylim=[0,10], figname ='median_sigma_err_t')
    display_comparison_plot_mpld3(it, err_q, names=['epnp','dls','p3p','ppnp','posit','lhm'], line_styles =['-', '--', '--', ':','-','-'], title='Median Rotation Error Plot (rad)', xtitle=r'\sigma', ytitle=r'$\tilde{e}_q$', ylim=[0,0.5], figname ='median_sigma_err_q')


    return err    

err_plot()
err_statistics_fcn_sigma()
err_statistics_fcn_n()
