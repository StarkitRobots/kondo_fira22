import math

f_step, s_step, rotation = 1, 1, math.pi / 2 #constant values: max forward step, max side step, max rotation
x_ret, y_ret, theta_ret = 0, 0, 0 
accuracy = 0.001 #desired accuracy


def approach(x_r, y_r, theta_r, x_b, y_b, theta_b): #robot's coordinates, ball's coordinates
    path = [[x_r, y_r, theta_r]]

    while [x_b, y_b, theta_b] != [x_r, y_r, theta_r]:
        try:
            alpha = math.atan(math.fabs((y_b - y_r) / (x_b - x_r))) #steering angle
            distance = math.sqrt((x_r - x_b) ** 2 + (y_r - y_b) ** 2) #distance to desired point
        except ZeroDivisionError:
            alpha, distance = 0, 0
        if y_b - y_r > accuracy:
            theta = alpha
        else:
            theta = alpha * (-1)
        if math.fabs(theta - theta_r) > accuracy and math.fabs(x_r - x_b) > accuracy: #turn to the ball
            if math.fabs(theta - theta_r) >= rotation:
                if theta - theta_r > 0:
                    theta_r += rotation
                else:
                    theta_r -= rotation
            else:
                theta_r = theta
        elif math.fabs(theta - theta_r) <= accuracy and math.fabs(x_r - x_b) > accuracy: #walk to the ball
            if y_r - y_b > accuracy:
                if x_r - x_b > accuracy:
                    if distance - f_step > accuracy:
                        x_r -= f_step * math.cos(alpha)
                        y_r -= f_step * math.sin(alpha)
                    else:
                        x_r = x_b
                        y_r = y_b
                else:
                    if distance - f_step > accuracy:
                        x_r += f_step * math.cos(alpha)
                        y_r -= f_step * math.sin(alpha)
                    else:
                        x_r = x_b
                        y_r = y_b
            elif y_b - y_r > accuracy:
                if x_r - x_b > accuracy:
                    if distance - f_step > accuracy:
                        x_r -= f_step * math.cos(alpha)
                        y_r += f_step * math.sin(alpha)
                    else:
                        x_r = x_b
                        y_r = y_b
                else:
                    if distance - f_step > accuracy:
                        x_r += f_step * math.cos(alpha)
                        y_r += f_step * math.sin(alpha)
                    else:
                        x_r = x_b
                        y_r = y_b
        elif math.fabs(theta - theta_r) > accuracy and math.fabs(x_r - x_b) <= accuracy: #turn to needed orientation
            if theta_b - theta_r > 0:
                if theta_b - theta_r - rotation > accuracy:
                    theta_r += rotation
                else:
                    theta_r = theta_b
            else:
                if theta_r - theta_b - rotation > accuracy:
                    theta_r -= rotation
                else:
                    theta_r = theta_b
        path.append([x_r, y_r, theta_r])
    return path
