import uaibot as ub
import numpy as np
from svg2gcode_py import *

def get_configuration(robot):
  return robot.q

def set_configuration_speed(robot, qdot_des):
  q_next = robot.q + qdot_des * dt
  robot.add_ani_frame(time = t + dt, q = q_next)

# Cria a função F:
# def F_func(r):
#     K = 5
#     F = np.matrix(np.zeros((4,1)))

#     F[0:3,0] = -K * r[0:3,0]
#     # F[0,0] = -K * np.sqrt(np.abs(r[0,0]))
#     # F[1,0] = -K * np.sqrt(np.abs(r[1,0]))
#     # F[2,0] = -K * np.sqrt(np.abs(r[2,0]))
#     F[3,0] = -K * np.sqrt(np.abs(r[3,0]))

#     return F

def F_func(r):
    A = [0.4, 0.4, 0.4, 2]
    w_tol = [0.08, 0.08, 0.08, 0.08]
    F = np.matrix(np.zeros((4, 1)))
    for i in range(4):
        if abs(r[i, 0]) < w_tol[i]:
            F[i, 0] = -A[i] * (r[i, 0] / w_tol[i])
        elif r[i, 0] >= w_tol[i]:
            F[i, 0] = -A[i]
        else:
            F[i, 0] = A[i]
    return F

def r_func(htm_e, htm_d):
    r = np.matrix(np.zeros((4,1)))

    r[0:3] = htm_e[0:3, 3] - htm_d[0:3, 3]
    r[3] = 1 - htm_d[0:3, 2].T * htm_e[0:3, 2]

    return r


def Jr_func(Jg, htm_d):
    Jr = np.matrix(np.zeros((4,n)))

    Jr[0:3,:] = Jg[0:3,:]
    Jr[3,:] = htm_d[0:3, 2].T * ub.Utils.S(fk[0:3, 2]) * Jg[3:6,:]

    return Jr

def dist_plane(point, normal, origin):
    d = np.abs(normal[0]*point[0] + normal[1]*point[1] + normal[2]*point[2] + (-normal[0]*origin[0] - normal[1]*origin[1] - normal[2]*origin[2]))
    d = d / np.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)
    return d


# sim = ub.Demo.control_demo_1()

image = "./imagens/batman2.svg"

img_scale = 1/500

# Position of image in the world
img_htm = [ [1, 0, 0, 0],
            [0, 0, -1, -0.4],
            [0, 1, 0, 0.4],
            [0, 0, 0,   1]]

img_htm = img_htm * ub.Utils.roty(-np.pi/4)

# Generate gcode for the image
gcode  = generate_gcode(image);

# Point array
shapes = []
shapes.append([])
# points = []

shape_index = 0
first_shape = True
# Render gcode points to point cloud
for line in gcode.split("\n"):
    if(line.startswith("G0")):
        # separate coordinates from line
        line = line.split(" ")
        # get coordinates
        point = []
        point.append(float(line[1][1:]) * img_scale)
        point.append(float(line[2][1:]) * img_scale)
        point.append(0)
        shapes[shape_index].append(point)

    elif line.startswith("M05"):
        if first_shape == False:
            shape_index += 1
        first_shape = False
        shapes.append([])



# Image axis 
img_axis = ub.Frame(img_htm, "img_axis")

# create the simulation
sim = ub.Simulation()
# Create kuka robot
robot = ub.Robot.create_kuka_kr5()
sim.add(robot)

# Add plane to img_htm position
plane = ub.Box(htm = img_htm * ub.Utils.trn([0, 0, 0.005]), width = 1, height = 0.001, depth = 1, name = "plane", color = "#555555")
sim.add(plane)

# Add pen box
pen_box_htm = [ [1, 0, 0, 0.4],
                [0, 1, 0, 0.4],
                [0, 0, 1, 0.05],
                [0, 0, 0,   1]]
pen_box = ub.Box(htm = pen_box_htm, width = 0.4, height = 0.1, depth = 0.4, color = "#222222")
sim.add(pen_box)

# Add pens
pen_colors = ["blue", "green", "red", "cyan", "yellow", "magenta", "white", "brown", "black"]
pens = []
pens_htm = []
point_clouds = []
for i in range(3):
    for k in range(3):
        pens_htm.append(pen_box_htm * ub.Utils.trn([-0.1 + 0.1 * i, -0.1 + 0.1 * k, 0.075]))
        pens.append(ub.Cylinder(htm = pens_htm[i*3+k], radius = 0.005, height = 0.05, color = pen_colors[i*3+k], name = ("pen" + str(i*3+k))))
        sim.add(pens[i*3+k])

        # point clouds
        point_clouds.append(ub.PointCloud(points = [[0],[0],[0]], size = 0.01, color = pen_colors[i*3+k], name = ("pc" + str(i*3+k))))
        sim.add(point_clouds[i*3+k])

# pen_1_htm = pen_box_htm * ub.Utils.trn([0, 0, 0.075])
# pen_1 = ub.Cylinder(htm = pen_1_htm, radius = 0.005, height = 0.05, color = "blue")

# points_htm.insert(0, pen_1_htm * ub.Utils.roty(np.pi) * ub.Utils.trn([0, 0, 0.025]))


# Point array transformed to img_axis
points_trn = []
points_htm = []

# Build task list
shape_index = 0
for shape in shapes:

    # get the right pen
    points_htm.append(pens_htm[shape_index] * ub.Utils.roty(np.pi) * ub.Utils.trn([0, 0, 0.025]))

    for point in shape:
        htm_aux = ub.Utils.trn(point)
        htm_aux = img_htm * htm_aux
        point_trn = [htm_aux[0,3], htm_aux[1,3], htm_aux[2,3]]
        points_trn.append(point_trn)
        points_htm.append(htm_aux)

    # return the pen to position
    points_htm.append(pens_htm[shape_index] * ub.Utils.roty(np.pi) * ub.Utils.trn([0, 0, 0.025]))

    shape_index += 1


# Effector position
htm_d = points_htm[0] * ub.Utils.trn([0, 0, -0.05])

# Captura o número de juntas do robô
n = np.shape(robot.q)[0]

dt = 0.01
t = 0
tmax = 6
#Colocaremos aqui nosso "main" do controlador, que ficará em um laço
#durante um tempo tmax

shape_index = 0
has_pen = False
has_drawn = False
shape_points = []
point_index = 0
while point_index < len(points_htm):
    #################################
    # Início da lógica de controle  #
    #################################

    # Measure the current configuration
    q = get_configuration(robot)

    # Get the Geometric Jacobian and Foward Kinematics
    Jg, fk = robot.jac_geo(q)

    # Calculate Task vector
    r = r_func(fk, htm_d)

    # Calculate Task Jacobian
    Jr = Jr_func(Jg, htm_d)

    # Calculate control action
    u = ub.Utils.dp_inv(Jr, 0.001) * F_func(r)

    # Actuate the robot
    set_configuration_speed(robot, u)

    # Paint the point on the plane
    pen_tip_htm = fk * ub.Utils.trn([0, 0, 0.05])
    current_point = [pen_tip_htm[0,3], pen_tip_htm[1,3], pen_tip_htm[2,3]]

    if dist_plane(current_point, -img_htm[0:3, 2], img_htm[0:3, 3]) < 0.001:
        pen_tip_htm = fk * ub.Utils.trn([0, 0, 0.05])
        shape_points.append(current_point)
        point_clouds[shape_index]._points = np.transpose(shape_points)
        point_clouds[shape_index].add_ani_frame(time = t + dt, initial_ind = 0, final_ind = len(shape_points) - 1)
        has_drawn = True

    # Pick up the pen
    if has_pen == False and has_drawn == False:
        dist_pen = [fk[0:3, 3] - pens_htm[shape_index][0:3, 3]]
        if np.linalg.norm(dist_pen) < 0.03:
            robot.attach_object(pens[shape_index])
            has_pen = True

    # Put down the pen
    if has_pen == True and has_drawn == True:
        dist_pen = [fk[0:3, 3] - pens_htm[shape_index][0:3, 3]]
        if np.linalg.norm(dist_pen) < 0.03:
            robot._attached_objects = []
            # print("detached")
            has_pen = False
            has_drawn = False
            shape_points = []
            shape_index += 1

    # iterate to next point
    if np.linalg.norm(r) < 1e-3:
        htm_d = points_htm[point_index] * ub.Utils.trn([0, 0, -0.05])
        point_index += 1
        # progress bar
        print("\rProgress: [{0:50s}] {1:.1f}%".format('#' * int(point_index / (len(points_htm) - 1) * 50), point_index / (len(points_htm) - 1) * 100), end="", flush=True)

    #################################
    # Fim da lógica de controle     #
    #################################

    #O tempo sempre vai passar no final do ciclo
    t += dt

#Vamos ver o resultado
# print(shape_points)
# print(point_cloud._points)

print()
sim.run()
sim.save('.\\','drawing_sim')
