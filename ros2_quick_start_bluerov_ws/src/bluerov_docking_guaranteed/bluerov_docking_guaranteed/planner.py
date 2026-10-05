import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from messages.srv import Wset

import numpy as np
import time

from codac import *


def make_visibility_separator(Xp, rho, theta):
    Z = VectorVar(2)

    f = AnalyticFunction(
        [Z],[ sqrt(sqr(Z[0] - Xp[0]) + sqr(Z[1] - Xp[1])),
              atan2(Z[1] - Xp[1], Z[0] - Xp[0])-Xp[2]])
    
    return SepInverse(f, [rho, theta])

def test_Xp(Xp_box, Xt_box, rho, theta, L_obs, eps=0.2):
    sep_fov = make_visibility_separator(Xp_box, rho, theta)
    sep_vis = SepVisible(Xp_box.subvector(0,1), L_obs)

    sep_inter = sep_vis & sep_fov

    # Pavage de l'objet avec le complément
    paving = pave(Xt_box, sep_inter, eps)

    Lin = paving.boxes(PavingInOut.inner)
    Lbound = paving.boxes(PavingInOut.bound)

    test_unknown = IntervalVector.empty(Xt_box.size())
    test_in      = IntervalVector.empty(Xt_box.size())
    for xv in Lin:
        test_in = test_in | (xv & Xt_box)
        test_unknown = test_unknown | (xv & Xt_box)

    if test_in == Xt_box: return "in"     # il existe un point non visible → échec

    for xj in Lbound: 
        test_unknown = test_unknown | (xj & Xt_box)
    
    if test_unknown != IntervalVector.empty(Xt_box.size()): return "maybe"   # indécidable à cette résolution
    

    else: return "out"      # visibilité complète prouvée

def W_set(Xp_domain, Xt, rho, theta, L_obs, eps=0.1):
    W_in, W_out, W_maybe = [], [], []

    L = [Xp_domain]
    while L:
        box = L.pop()

        status = test_Xp(box, Xt.subvector(0,1), rho, theta, L_obs, eps)

        if status == "in":
            W_in.append(box)
        elif status == "out":
            W_out.append(box)
        else:
            if box.max_diam() < eps:
                W_maybe.append(box)
            else:
                i = box.subvector(0,2).max_diam_index()
                b1, b2 = box.bisect(i)
                L.append(b1)
                L.append(b2)

    return W_in, W_out, W_maybe

def W_set_quick(Xp_domain, Xt, rho, theta, L_obs, eps=0.1):
    W_in, W_out, W_maybe = [], [], []

    L = [Xp_domain]
    while L:
        box = L.pop()

        status = test_Xp(box, Xt.subvector(0,1), rho, theta, L_obs, 4*eps)

        if status == "in":
            return [box], W_out, W_maybe
        elif status == "out":
            W_out.append(box)
        else:
            if box.max_diam() < eps:
                W_maybe.append(box)
            else:
                i = box.subvector(0,2).max_diam_index()
                b1, b2 = box.bisect(i)
                L.append(b1)
                L.append(b2)

    return W_in, W_out, W_maybe

def draw_Wset3D(fig, W_in, W_out, W_maybe, Xt, Xp_domain, display_maybe=True, display_biggest_box=True):
    fig.draw_axes(1.0)
    fig.draw_box(Xt, Color.blue(0.5))

    for b in W_in: fig.draw_box(b, Color.green(0.4))
    if display_maybe:
        for b in W_maybe: fig.draw_box(b, Color.yellow(0.8))
    
def find_centre_biggest_box(Lbox):
    
    k_max,Vmax = 0,0
    for k, b in enumerate(Lbox):
        volume_box = 1
        for n in range(len(Lbox[0])):
            volume_box *= b[n].diam()
        if Vmax < volume_box:
            Vmax = volume_box
            k_max = k
        
    return Lbox[k_max]

def find_centre_closest_box(Lbox,pose):
    x,y,psi = pose
    x0,y0,psi0 = Lbox[0].mid()
    k_min,dmin = 0,np.sqrt((x-x0)**2+(y-y0)**2)
    
    for k, b in enumerate(Lbox):
        xk,yk,psik = Lbox[k].mid()
        distance_box = np.sqrt((x-xk)**2+(y-yk)**2)
        if dmin > distance_box:
            dmin = distance_box
            k_min = k
        
    return Lbox[k_min]

def computeWset(Xp_domain, Xt, rho, theta, pose, L_obs, L_contour, display="all"):
    # ------- Calcul de W_set -------
    #W_in, W_out, W_maybe = W_set(Xp_domain, Xt, rho, theta, L_obs, eps=0.4)
    W_in, W_out, W_maybe = W_set_quick(Xp_domain, Xt, rho, theta, L_obs, eps=0.4)
    if W_in == []:
        print("W_in is empty, try increasing eps")
        return None
    else:
        closest_box = find_centre_closest_box(W_in,pose)
        print(f"Closest box center: {closest_box.mid()}")
        
        if display in ["all", "2D"]:
            fig2D = Figure2D("Wset2D", GraphicOutput.VIBES | GraphicOutput.IPE)
            for b in W_in: fig2D.draw_box(b.subvector(0,1), StyleProperties.inside())
            for s in L_obs: 
                fig2D.draw_line(s, StyleProperties(Color.red(),"w:0.05","z:5"))
            for s in L_contour: 
                fig2D.draw_line(s, StyleProperties(Color.black(0.5),"w:0.05","z:5"))
            fig2D.draw_box(Xt.subvector(0,1), StyleProperties(Color.blue(0.5),"w:0.05","z:5"))

        if display in ["all", "3D"]:
            fig3D = Figure3D("Wset3D")
            draw_Wset3D(fig3D, W_in, W_out, W_maybe, Xt, Xp_domain, display_maybe=False)
            draw_3Dobstacles(fig3D, L_obs, N=100, color=Color.red(0.2))
            draw_3Dobstacles(fig3D, L_contour, N=100, color=Color.gray(0.2))

            fig3D.draw_box(closest_box, Color.pink(0.8))
            fig3D.draw_box(IntervalVector(pose).inflate(0.1), Color.pink(0.8))
            draw_Wset3D(fig3D, W_in, W_out, W_maybe, Xt, Xp_domain, display_maybe=False)
        
        return closest_box.mid()
            
def draw_3Dobstacles(fig, L_obs, N=100, color=Color.red(0.2)):
    for s in L_obs:
        x0,y0 = s[0]
        x1,y1 = s[1]
        for k in range(N):
            xk,yk = x0 + k/N*(x1-x0), y0 + k/N*(y1-y0)
            box = IntervalVector(3)
            box[0] = Interval(xk).inflate(0.01)
            box[1] = Interval(yk).inflate(0.01)
            box[2] = Interval(-np.pi,np.pi)
            fig.draw_box(box, color)  



class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')
        self.ns = self.get_namespace()
        # -------- Paramètres du problème --------
        self.x_min, self.x_max = 0.5, 19.5
        self.y_min, self.y_max = 0.5, 11.5
        self.psi_min, self.psi_max = -np.pi, np.pi

        self.rmin, self.rmax = 0.3, 5.0
        self.FOV = np.deg2rad(60)

        self.L_obs = []#[Segment([0.0, 0.7], [0.95, 1.5]),Segment([1.2, 0.55], [2.62, 0.95])]

        self.L_contour = [Segment([self.x_min, self.y_min], [self.x_min, self.y_max]),Segment([self.x_min, self.y_max], [self.x_max, self.y_max]),Segment([self.x_max, self.y_max], [self.x_max, self.y_min]),Segment([self.x_max, self.y_min], [self.x_min, self.y_min]),]


        self.delta = 1.0 # incertitude sur la position du target

        # Service
        self.srv = self.create_service(
            Wset,
            self.ns+'/wset_compute',
            self.compute_callback
        )
        self.pursuer_desired_pose  = self.create_publisher(Pose, self.ns+'/pursuer_desired_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_callback)

        self.resulting_pose = Pose()
        self.get_logger().info("Planner node ready.")

    def publish_callback(self):
        self.pursuer_desired_pose.publish(self.resulting_pose)

    # -------- CALLBACK SERVICE --------
    def compute_callback(self, request, response):
        self.get_logger().info(f"Request received")
        # pose_pursuer
        xp = request.pose_pursuer.position.x
        yp = request.pose_pursuer.position.y
        yawp = self.yaw_from_pose(request.pose_pursuer)

        # pose_target
        xt = request.pose_target.position.x
        yt = request.pose_target.position.y
        yawt = self.yaw_from_pose(request.pose_target)

        # -------- Interval setup --------
        Xp_domain = IntervalVector(3)
        Xp_domain[0] = Interval(self.x_min, self.x_max)
        Xp_domain[1] = Interval(self.y_min, self.y_max)
        Xp_domain[2] = Interval(self.psi_min, self.psi_max)

        Xt = IntervalVector(3)
        Xt[0] = Interval(xt - self.delta, xt + self.delta)
        Xt[1] = Interval(yt - self.delta, yt + self.delta)
        Xt[2] = Interval(-np.pi, np.pi)

        theta = Interval(-self.FOV/2, self.FOV/2)
        rho = Interval(self.rmin, self.rmax)

        pose_pursuer = [xp, yp, yawp]

        # -------- LONG COMPUTE --------
        x, y, yaw = computeWset(Xp_domain, Xt, rho, theta, pose_pursuer, self.L_obs, self.L_contour, display="all")

        # -------- RESPONSE --------
        self.resulting_pose = Pose()
        self.resulting_pose.position.x = float(x)
        self.resulting_pose.position.y = float(y)
        self.resulting_pose.position.z = float(-1.5)

        # quaternion from yaw
        self.resulting_pose.orientation.z = np.sin(yaw / 2.0)
        self.resulting_pose.orientation.w = np.cos(yaw / 2.0)

        response.pose_result = self.resulting_pose

        return response

    def yaw_from_pose(self, pose):
        # simple extraction (suppose quaternion z-w only)
        z = pose.orientation.z
        w = pose.orientation.w
        return np.arctan2(2*w*z, 1 - 2*z*z)


def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()