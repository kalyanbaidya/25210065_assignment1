import tkinter as tk
import math
import time

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.last_error = 0
        self.last_time = None

    def update(self, current_value):
        error = self.setpoint - current_value
        current_time = time.time()
        delta_time = current_time - self.last_time if self.last_time else 0
        self.last_time = current_time

        if delta_time > 0:
            self.integral += error * delta_time
            derivative = (error - self.last_error) / delta_time
        else:
            derivative = 0

        self.last_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output

class InvertedPendulumApp:
    def __init__(self, root):
        self.root = root
        self.width = 600
        self.height = 400
        self.canvas = tk.Canvas(root, width=self.width, height=self.height, bg='white')
        self.canvas.pack()

        # Pendulum parameters
        self.cart_x = self.width / 2
        self.cart_y = self.height - 100
        self.cart_width = 80
        self.cart_height = 30

        self.pendulum_length = 150
        self.theta = math.pi / 4  # start tilted 45 degrees
        self.omega = 0  # angular velocity
        self.alpha = 0  # angular acceleration

        # Physics constants
        self.g = 40
        self.mass_pendulum = 1.0
        self.mass_cart = 2.0
        self.dt = 0.02  # timestep

        # Control (PID) - increased gains for more responsiveness
        self.pid = PID(Kp=200, Ki=0, Kd=100, setpoint=0)  # angle setpoint is 0 radians (upright)

        # For dragging
        self.dragging = False

        # Initialize cart velocity
        self.cart_velocity = 0

        # Bind events
        self.canvas.bind("<ButtonPress-1>", self.start_drag)
        self.canvas.bind("<B1-Motion>", self.drag)
        self.canvas.bind("<ButtonRelease-1>", self.end_drag)

        self.update()

    def start_drag(self, event):
        # Check if clicked near pendulum bob
        bob_x, bob_y = self.get_pendulum_pos()
        dist = math.hypot(event.x - bob_x, event.y - bob_y)
        if dist < 20:
            self.dragging = True

    def drag(self, event):
        if self.dragging:
            # Calculate new angle based on mouse position relative to cart
            dx = event.x - self.cart_x
            dy = event.y - self.cart_y
            self.theta = math.atan2(dx, -dy)  # angle from vertical downward
            self.omega = 0  # reset angular velocity while dragging

    def end_drag(self, event):
        self.dragging = False

    def get_pendulum_pos(self):
        # Position of pendulum bob
        x = self.cart_x + self.pendulum_length * math.sin(self.theta)
        y = self.cart_y - self.pendulum_length * math.cos(self.theta)
        return x, y

    def physics_step(self, force):
        # Equations of motion for inverted pendulum on a cart
        M = self.mass_cart
        m = self.mass_pendulum
        l = self.pendulum_length
        g = self.g
        theta = self.theta
        omega = self.omega

        # Angular acceleration (from dynamics equations)
        numerator = g * math.sin(theta) + math.cos(theta) * (-force - m * l * omega**2 * math.sin(theta)) / (M + m)
        denominator = l * (4/3 - m * math.cos(theta)**2 / (M + m))
        alpha = numerator / denominator

        # Update angular velocity and angle
        omega += alpha * self.dt
        theta += omega * self.dt

        # Update cart position and velocity
        cart_acc = force / (M + m)

        # Clamp acceleration to avoid instability
        max_acc = 100000
        if cart_acc > max_acc:
            cart_acc = max_acc
        elif cart_acc < -max_acc:
            cart_acc = -max_acc

        self.cart_velocity += cart_acc * self.dt

        # Clamp cart velocity for smoothness
        max_vel = 10000
        if self.cart_velocity > max_vel:
            self.cart_velocity = max_vel
        elif self.cart_velocity < -max_vel:
            self.cart_velocity = -max_vel

        self.cart_x += self.cart_velocity * self.dt

        # Limit cart movement within canvas
        self.cart_x = max(self.cart_width / 2, min(self.width - self.cart_width / 2, self.cart_x))

        self.theta = theta
        self.omega = omega
        self.alpha = alpha

    def update(self):
        if not self.dragging:
            force = -self.pid.update(self.theta)
            # Remove damping for faster response
            # self.cart_velocity *= 0.9

            self.physics_step(force)
        else:
            # Reset cart velocity when dragging for smooth control
            self.cart_velocity = 0

        self.draw()
        # Smaller delay for more fluid updates
        self.root.after(int(self.dt * 1000), self.update)

    def draw(self):
        self.canvas.delete("all")
        # Draw cart
        x0 = self.cart_x - self.cart_width / 2
        y0 = self.cart_y - self.cart_height / 2
        x1 = self.cart_x + self.cart_width / 2
        y1 = self.cart_y + self.cart_height / 2
        self.canvas.create_rectangle(x0, y0, x1, y1, fill='blue')

        # Draw pendulum rod
        bob_x, bob_y = self.get_pendulum_pos()
        self.canvas.create_line(self.cart_x, self.cart_y, bob_x, bob_y, width=4, fill='black')

        # Draw pendulum bob
        self.canvas.create_oval(bob_x - 15, bob_y - 15, bob_x + 15, bob_y + 15, fill='red')

        # Draw ground line
        self.canvas.create_line(0, self.cart_y + self.cart_height / 2, self.width, self.cart_y + self.cart_height / 2, fill='grey')

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Inverted Pendulum with PID Control - Fast Response")
    app = InvertedPendulumApp(root)
    root.mainloop()
