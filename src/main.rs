use nannou::prelude::*;

// DEFAULT PARAMS
// TURNFACTOR: Controls how much boids can turn in response to their neighbors.
const TURNFACTOR: f32 = 0.2;

// VISUAL_RANGE: Defines the maximum distance at which boids can detect and react to other boids.
const VISUAL_RANGE: f32 = 40.0;

// PROTECTED_RANGE: Specifies the minimum distance to maintain from other boids to avoid collisions.
const PROTECTED_RANGE: f32 = 8.0;

// CENTERING_FACTOR: Determines how strongly boids are attracted to the center of the flock.
const CENTERING_FACTOR: f32 = 0.0005;

// AVOID_FACTOR: Controls how much boids avoid collisions with each other.
const AVOID_FACTOR: f32 = 0.05;

// MATCHING_FACTOR: Controls how much boids attempt to match the velocity of their neighbors.
const MATCHING_FACTOR: f32 = 0.05;

// MAX_SPEED: Sets the maximum speed at which boids can move.
const MAX_SPEED: f32 = 6.0;

// MIN_SPEED: Sets the minimum speed below which boids won't slow down further.
const MIN_SPEED: f32 = 3.0;

// MAX_BIAS: Specifies the maximum bias (a parameter for behavioral variation) for boids.
const MAX_BIAS: f32 = 0.01;

// BIAS_INCREMENT: Determines the rate at which bias values change over time.
const BIAS_INCREMENT: f32 = 0.00004;

// DEFAULT_BIAS_VAL: Sets the initial bias value for boids.
const DEFAULT_BIAS_VAL: f32 = 0.001;

const NUM_BOIDS: usize = 150;
const BOUNDARY_MARGIN: f32 = 80.0;

struct Params {
    turnfactor: f32,
    visual_range: f32,
    protected_range: f32,
    centering_factor: f32,
    avoid_factor: f32,
    matching_factor: f32,
    max_speed: f32,
    min_speed: f32,
    max_bias: f32,
    bias_increment: f32,
    default_bias_val: f32,
}

#[derive(Clone, Copy, PartialEq)]
enum ScoutGroup {
    None,
    Right,
    Left,
}

#[derive(Clone, Copy)]
struct Boid {
    position: Vec2,
    velocity: Vec2,
    scout_group: ScoutGroup,
    bias_val: f32,
}

struct Model {
    boids: Vec<Boid>,
    params: Params,
}

fn model(app: &App) -> Model {
    app.new_window().size(1000, 700).view(view).build().unwrap();

    let params = Params {
        turnfactor: TURNFACTOR,
        avoid_factor: AVOID_FACTOR,
        bias_increment: BIAS_INCREMENT,
        centering_factor: CENTERING_FACTOR,
        default_bias_val: DEFAULT_BIAS_VAL,
        visual_range: VISUAL_RANGE,
        protected_range: PROTECTED_RANGE,
        matching_factor: MATCHING_FACTOR,
        max_speed: MAX_SPEED,
        min_speed: MIN_SPEED,
        max_bias: MAX_BIAS,
    };

    let mut boids = Vec::with_capacity(NUM_BOIDS);

    for i in 0..NUM_BOIDS {
        let position = pt2(
            random_range(-400.0, 400.0),
            random_range(-250.0, 250.0),
        );
        let angle = random_range(0.0, PI * 2.0);
        let speed = random_range(params.min_speed, params.max_speed);
        let velocity = vec2(angle.cos() * speed, angle.sin() * speed);

        let scout_group = if i < NUM_BOIDS / 10 {
            ScoutGroup::Right
        } else if i < (NUM_BOIDS * 2) / 10 {
            ScoutGroup::Left
        } else {
            ScoutGroup::None
        };

        boids.push(Boid {
            position,
            velocity,
            scout_group,
            bias_val: params.default_bias_val,
        });
    }

    Model {
        boids,
        params,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    let boundary = app.window_rect();
    let params = &model.params;

    let visual_range_sq = params.visual_range * params.visual_range;
    let protected_range_sq = params.protected_range * params.protected_range;

    // Snapshot of previous frame state so all boid interactions are evaluated concurrently
    let boid_snapshots = model.boids.clone();

    for (i, boid) in model.boids.iter_mut().enumerate() {
        let mut xpos_avg = 0.0;
        let mut ypos_avg = 0.0;
        let mut xvel_avg = 0.0;
        let mut yvel_avg = 0.0;
        let mut neighboring_boids = 0.0;
        let mut close_dx = 0.0;
        let mut close_dy = 0.0;

        for (j, other) in boid_snapshots.iter().enumerate() {
            if i == j {
                continue;
            }

            let diff = boid.position - other.position;
            if diff.x.abs() < params.visual_range && diff.y.abs() < params.visual_range {
                let squared_dist = diff.length_squared();

                if squared_dist < protected_range_sq {
                    // Separation: accumulate difference from nearby boids
                    close_dx += diff.x;
                    close_dy += diff.y;
                } else if squared_dist < visual_range_sq {
                    // Cohesion and Alignment: accumulate neighbors' position and velocity
                    xpos_avg += other.position.x;
                    ypos_avg += other.position.y;
                    xvel_avg += other.velocity.x;
                    yvel_avg += other.velocity.y;
                    neighboring_boids += 1.0;
                }
            }
        }

        // Apply Cohesion and Alignment
        if neighboring_boids > 0.0 {
            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;
            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;

            boid.velocity.x += (xpos_avg - boid.position.x) * params.centering_factor
                + (xvel_avg - boid.velocity.x) * params.matching_factor;
            boid.velocity.y += (ypos_avg - boid.position.y) * params.centering_factor
                + (yvel_avg - boid.velocity.y) * params.matching_factor;
        }

        // Apply Separation
        boid.velocity.x += close_dx * params.avoid_factor;
        boid.velocity.y += close_dy * params.avoid_factor;

        // Apply soft boundary steering
        if boid.position.x < boundary.left() + BOUNDARY_MARGIN {
            boid.velocity.x += params.turnfactor;
        }
        if boid.position.x > boundary.right() - BOUNDARY_MARGIN {
            boid.velocity.x -= params.turnfactor;
        }
        if boid.position.y < boundary.bottom() + BOUNDARY_MARGIN {
            boid.velocity.y += params.turnfactor;
        }
        if boid.position.y > boundary.top() - BOUNDARY_MARGIN {
            boid.velocity.y -= params.turnfactor;
        }

        // Apply Scout bias
        match boid.scout_group {
            ScoutGroup::Right => {
                if boid.velocity.x > 0.0 {
                    boid.bias_val = (boid.bias_val + params.bias_increment).min(params.max_bias);
                } else {
                    boid.bias_val = (boid.bias_val - params.bias_increment).max(params.bias_increment);
                }
                boid.velocity.x = (1.0 - boid.bias_val) * boid.velocity.x + boid.bias_val * 1.0;
            }
            ScoutGroup::Left => {
                if boid.velocity.x < 0.0 {
                    boid.bias_val = (boid.bias_val + params.bias_increment).min(params.max_bias);
                } else {
                    boid.bias_val = (boid.bias_val - params.bias_increment).max(params.bias_increment);
                }
                boid.velocity.x = (1.0 - boid.bias_val) * boid.velocity.x + boid.bias_val * (-1.0);
            }
            ScoutGroup::None => {}
        }

        // Clamp speed between min_speed and max_speed
        let speed = boid.velocity.length();
        if speed > params.max_speed {
            boid.velocity = (boid.velocity / speed) * params.max_speed;
        } else if speed < params.min_speed {
            if speed > 0.0001 {
                boid.velocity = (boid.velocity / speed) * params.min_speed;
            } else {
                boid.velocity = vec2(params.min_speed, 0.0);
            }
        }

        // Update position
        boid.position += boid.velocity;
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(WHITE);

    for boid in &model.boids {
        let angle = boid.velocity.y.atan2(boid.velocity.x);
        let color = match boid.scout_group {
            ScoutGroup::Right => rgb(0.2, 0.4, 0.8),
            ScoutGroup::Left => rgb(0.8, 0.3, 0.3),
            ScoutGroup::None => rgb(0.1, 0.1, 0.1),
        };

        // Draw streamlined boid pointing along its velocity vector
        draw.ellipse()
            .xy(boid.position)
            .w_h(12.0, 6.0)
            .rotate(angle)
            .color(color);
    }

    draw.to_frame(app, &frame).unwrap();
}

fn main() {
    nannou::app(model).update(update).view(view).run();
}
