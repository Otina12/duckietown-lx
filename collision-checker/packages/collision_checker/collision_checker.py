import itertools
import math
from typing import List, Tuple

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            environment=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(
    environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # Rototranslate the robot_body by the robot_pose
    rototranslated_robot = rototranslate_robot_body(robot_body, robot_pose)
    
    # Check if the robot collides with the environment
    return check_collision_list(rototranslated_robot, environment)


def rototranslate_robot_body(
    robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> List[PlacedPrimitive]:
    """Apply the robot pose transformation to each part of the robot body."""
    rototranslated_robot = []
    
    for part in robot_body:
        # Create a new pose that combines the robot's global pose with the part's local pose
        new_pose = combine_poses(robot_pose, part.pose)
        
        # Create a new PlacedPrimitive with the combined pose and the same primitive
        new_part = PlacedPrimitive(
            pose=new_pose,
            primitive=part.primitive,
            motion=part.motion,
            appearance=part.appearance
        )
        
        rototranslated_robot.append(new_part)
        
    return rototranslated_robot


def combine_poses(global_pose: FriendlyPose, local_pose: FriendlyPose) -> FriendlyPose:
    """Combine a global pose with a local pose."""
    # Convert global pose angle to radians
    global_theta_rad = math.radians(global_pose.theta_deg)
    
    # Calculate the rotated x, y coordinates
    cos_theta = math.cos(global_theta_rad)
    sin_theta = math.sin(global_theta_rad)
    
    # Rotate the local coordinates
    rotated_x = local_pose.x * cos_theta - local_pose.y * sin_theta
    rotated_y = local_pose.x * sin_theta + local_pose.y * cos_theta
    
    # Translate the rotated coordinates
    new_x = global_pose.x + rotated_x
    new_y = global_pose.y + rotated_y
    
    # Combine the angles
    new_theta_deg = (global_pose.theta_deg + local_pose.theta_deg) % 360
    
    return FriendlyPose(x=new_x, y=new_y, theta_deg=new_theta_deg)


def check_collision_list(
    rototranslated_robot: List[PlacedPrimitive], environment: List[PlacedPrimitive]
) -> bool:
    """Check if any robot part collides with any environment object."""
    for robot_part, env_object in itertools.product(rototranslated_robot, environment):
        if check_collision_shape(robot_part, env_object):
            return True
    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    """Check if two primitives are colliding."""
    # Handle Circle vs Circle
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        return check_circle_circle_collision(a, b)
    
    # Handle Rectangle vs Circle (both cases)
    elif isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        return check_rectangle_circle_collision(a, b)
    elif isinstance(a.primitive, Circle) and isinstance(b.primitive, Rectangle):
        return check_rectangle_circle_collision(b, a)
    
    # Handle Rectangle vs Rectangle
    elif isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        return check_rectangle_rectangle_collision(a, b)
    
    # Fallback (shouldn't happen with the given primitives)
    return False


def check_circle_circle_collision(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    """Check if two circles are colliding."""
    # Extract circles
    circle_a = a.primitive
    circle_b = b.primitive
    
    # Calculate distance between circle centers
    dx = a.pose.x - b.pose.x
    dy = a.pose.y - b.pose.y
    distance = math.sqrt(dx * dx + dy * dy)
    
    # Circles collide if the distance is less than the sum of their radii
    return distance < (circle_a.radius + circle_b.radius)


def check_rectangle_circle_collision(rect: PlacedPrimitive, circle: PlacedPrimitive) -> bool:
    """Check if a rectangle and a circle are colliding."""
    # Transform the circle's position into the rectangle's coordinate frame
    relative_pose = transform_to_local_frame(circle.pose, rect.pose)
    
    # Extract rectangle and circle
    rectangle = rect.primitive
    circ = circle.primitive
    
    # Find the closest point on the rectangle to the circle center
    closest_x = max(rectangle.xmin, min(relative_pose.x, rectangle.xmax))
    closest_y = max(rectangle.ymin, min(relative_pose.y, rectangle.ymax))
    
    # Calculate distance from closest point to circle center
    dx = relative_pose.x - closest_x
    dy = relative_pose.y - closest_y
    distance = math.sqrt(dx * dx + dy * dy)
    
    # Collision if the distance is less than the circle's radius
    return distance < circ.radius


def check_rectangle_rectangle_collision(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    """Check if two rectangles are colliding using the Separating Axis Theorem (SAT)."""
    # Get the corners of both rectangles in world coordinates
    corners_a = get_rectangle_corners_world(a)
    corners_b = get_rectangle_corners_world(b)
    
    # Get the axes to check for separation (normals of each rectangle's edges)
    axes = get_separating_axes(corners_a, corners_b)
    
    # Check for separation along each axis
    for axis in axes:
        # Project both rectangles onto the axis
        projection_a = project_onto_axis(corners_a, axis)
        projection_b = project_onto_axis(corners_b, axis)
        
        # If there's a separation along any axis, the rectangles don't collide
        if projection_a[1] < projection_b[0] or projection_b[1] < projection_a[0]:
            return False
    
    # No separation found, the rectangles collide
    return True


def transform_to_local_frame(pose: FriendlyPose, reference_pose: FriendlyPose) -> FriendlyPose:
    """Transform a pose from world coordinates to the reference pose's local coordinates."""
    # Calculate relative position
    dx = pose.x - reference_pose.x
    dy = pose.y - reference_pose.y
    
    # Convert reference angle to radians
    ref_theta_rad = math.radians(reference_pose.theta_deg)
    
    # Rotate the point to the reference frame
    cos_theta = math.cos(ref_theta_rad)
    sin_theta = math.sin(ref_theta_rad)
    
    local_x = dx * cos_theta + dy * sin_theta
    local_y = -dx * sin_theta + dy * cos_theta
    
    # Calculate relative angle
    local_theta_deg = (pose.theta_deg - reference_pose.theta_deg) % 360
    
    return FriendlyPose(x=local_x, y=local_y, theta_deg=local_theta_deg)


def get_rectangle_corners_world(rect_placed: PlacedPrimitive) -> List[Tuple[float, float]]:
    """Get the corners of a rectangle in world coordinates."""
    rect = rect_placed.primitive
    pose = rect_placed.pose
    
    # Define corners in local coordinates
    corners_local = [
        (rect.xmin, rect.ymin),  # Bottom-left
        (rect.xmax, rect.ymin),  # Bottom-right
        (rect.xmax, rect.ymax),  # Top-right
        (rect.xmin, rect.ymax),  # Top-left
    ]
    
    # Convert to world coordinates
    corners_world = []
    theta_rad = math.radians(pose.theta_deg)
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)
    
    for x_local, y_local in corners_local:
        # Rotate
        x_rotated = x_local * cos_theta - y_local * sin_theta
        y_rotated = x_local * sin_theta + y_local * cos_theta
        
        # Translate
        x_world = pose.x + x_rotated
        y_world = pose.y + y_rotated
        
        corners_world.append((x_world, y_world))
    
    return corners_world


def get_separating_axes(corners_a: List[Tuple[float, float]], corners_b: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Get the axes to check for separation in the SAT algorithm."""
    axes = []
    
    # Add the normals of the edges of both rectangles
    for corners in [corners_a, corners_b]:
        for i in range(4):
            # Get the current edge
            edge_start = corners[i]
            edge_end = corners[(i + 1) % 4]
            
            # Calculate the edge vector
            edge_x = edge_end[0] - edge_start[0]
            edge_y = edge_end[1] - edge_start[1]
            
            # Get the normal vector (perpendicular to the edge)
            normal_x = -edge_y
            normal_y = edge_x
            
            # Normalize the normal vector
            length = math.sqrt(normal_x * normal_x + normal_y * normal_y)
            if length > 0:
                normal_x /= length
                normal_y /= length
                
            axes.append((normal_x, normal_y))
    
    return axes


def project_onto_axis(corners: List[Tuple[float, float]], axis: Tuple[float, float]) -> Tuple[float, float]:
    """Project a rectangle's corners onto an axis and return the min and max projections."""
    min_proj = float('inf')
    max_proj = float('-inf')
    
    for corner in corners:
        # Calculate the dot product of the corner and axis
        projection = corner[0] * axis[0] + corner[1] * axis[1]
        
        min_proj = min(min_proj, projection)
        max_proj = max(max_proj, projection)
    
    return (min_proj, max_proj)