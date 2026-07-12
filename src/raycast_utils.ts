import type { rect_obstacle } from "./field";

export interface lidar_ray {
    start_pos: point_vector;
    end_pos: point_vector;
    radius: number;
    angle: number;
}

export interface point_vector {
    x: number;
    y: number;
}

export function check_wall_collision(startpoint: point_vector, endpoint: point_vector, angle: number): point_vector | null{
    let new_endpoint = { x: endpoint.x, y: endpoint.y };
    let slope = Math.tan(angle);
    let collision_true = false;
    
    if (new_endpoint.y > 320) {
        new_endpoint.y = 320;
        new_endpoint.x = startpoint.x + (320 - startpoint.y) / slope;
        collision_true = true;
    } else if (new_endpoint.y < -320) {
        new_endpoint.y = -320;
        new_endpoint.x = startpoint.x + (-320 - startpoint.y) / slope;
        collision_true = true;
    }

    if (new_endpoint.x > 320) {
        new_endpoint.x = 320;
        new_endpoint.y = startpoint.y + (320 - startpoint.x) * slope;
        collision_true = true;
    } else if (new_endpoint.x < -320) {
        new_endpoint.x = -320;
        new_endpoint.y = startpoint.y + (-320 - startpoint.x) * slope;
        collision_true = true;
    }
    
    if(collision_true){return new_endpoint;}
    return null;
}

export function get_distance(start: point_vector, end: point_vector){
    
    return Math.sqrt((start.x - end.x) ** 2 + (start.y - end.y) ** 2);
}

//slab method for line and rectangle collision
export function check_rect_collision(start: point_vector, angle: number, max_radius: number, box: rect_obstacle): point_vector | null{
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);

    let t1 = (box.min.x - start.x) / dx;
    let t2 = (box.max.x - start.x) / dx;
    let t3 = (box.min.y - start.y) / dy;
    let t4 = (box.max.y - start.y) / dy;

    let tMinX = Math.min(t1, t2);
    let tMaxX = Math.max(t1, t2);
    let tMinY = Math.min(t3, t4);
    let tMaxY = Math.max(t3, t4);

    let tMin = Math.max(tMinX, tMinY);
    let tMax = Math.min(tMaxX, tMaxY);

    if(tMax < tMin || tMax < 0){
        return null;
    }

    if (tMin > max_radius){
        return null;
    }

    let hitDistance = Math.max(tMin, 0);

    return {
        x: start.x + dx * hitDistance,
        y: start.y + dy * hitDistance
    }
}
