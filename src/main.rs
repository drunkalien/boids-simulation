use nannou::prelude::*;

// DEFAULT PARAMS
// TURNFACTOR: Controls how much boids can turn in response to their neighbors.
const TURNFACTOR: f32 = 0.2;

// VISUAL_RANGE: Defines the maximum distance at which boids can detect and react to other boids.
const VISUAL_RANGE: f32 = 40.0;

// PROTECTED_RANGE: Specifies the minimum distance to maintain from other boids to avoid collisions.
const PROTECTED_RANGE: f32 = 15.0;

// CENTERING_FACTOR: Determines how strongly boids are attracted to the center of the flock.
const CENTERING_FACTOR: f32 = 0.0005;

// AVOID_FACTOR: Controls how much boids avoid collisions with each other.
const AVOID_FACTOR: f32 = 0.005;

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

struct Boid {
    position: Vec2,
    velocity: Vec2,
    direction_x: DirectionX,
    direction_y: DirectionY,
}

impl Boid {
    fn separate(
        &mut self,
        boid_positions: &Vec<Vec2>,
        current_boid_index: usize,
        avoid_factor: f32,
        protected_range: f32,
        max_speed: f32,
    ) {
        let mut close_dx = 0.0f32;
        let mut close_dy = 0.0f32;
        for i in 0..boid_positions.len() {
            if i == current_boid_index {
                continue;
            }
            let other_boid_position = boid_positions[i];
            let distance = self.position - other_boid_position;
            if distance[0].abs() < protected_range || distance[1].abs() < protected_range {
                close_dx += self.position[0] - other_boid_position[0];
                close_dy += self.position[1] - other_boid_position[1];
            }
        }

        self.position[0] += close_dx * avoid_factor;
        self.position[1] += close_dy * avoid_factor;
    }
}

enum DirectionX {
    Right,
    Left,
}

enum DirectionY {
    Top,
    Bottom,
}

struct Model {
    _boids: Vec<Boid>,
    _window: WindowId,
    _params: Params,
}

fn model(_app: &App) -> Model {
    let window = _app.new_window().view(view).build().unwrap();
    let position = pt2(-100.0, 100.0);
    let velocity = pt2(4.0, 3.0);

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

    let mut boids = Vec::new();

    for i in 0..10 {
        let boid = Boid {
            position: pt2(position[0] + i as f32 * 10.0, position[1] + i as f32 * 30.0),
            velocity,
            direction_x: DirectionX::Right,
            direction_y: DirectionY::Top,
        };

        boids.push(boid);
    }

    Model {
        _boids: boids,
        _window: window,
        _params: params,
    }
}

fn update(_app: &App, _model: &mut Model, _update: Update) {
    let boundary = _app.window_rect();
    let boids = &mut _model._boids;

    let mut boid_positions_snapshot: Vec<Vec2> = Vec::new();

    for boid in boids {
        boid_positions_snapshot.push(boid.position);
    }

    let boids = &mut _model._boids;

    for i in 0..boids.len() {
        let boid = &mut boids[i];
        boid.separate(
            &boid_positions_snapshot,
            i,
            _model._params.avoid_factor,
            _model._params.protected_range,
            _model._params.max_speed,
        );

        if boid.velocity[0] > _model._params.max_speed {
            boid.velocity[0] = _model._params.max_speed;
        }

        if boid.velocity[1] > _model._params.max_speed {
            boid.velocity[1] = _model._params.max_speed;
        }
        if boid.position[0] >= boundary.right() - 10.0 {
            boid.direction_x = DirectionX::Left;
        } else if boid.position[0] <= boundary.left() + 10.0 {
            boid.direction_x = DirectionX::Right;
        }

        if boid.position[1] >= boundary.top() - 10.0 {
            boid.direction_y = DirectionY::Bottom;
        } else if boid.position[1] <= boundary.bottom() + 10.0 {
            boid.direction_y = DirectionY::Top;
        }

        match boid.direction_x {
            DirectionX::Left => boid.position[0] -= boid.velocity[0],
            DirectionX::Right => boid.position[0] += boid.velocity[0],
        }

        match boid.direction_y {
            DirectionY::Top => boid.position[1] += boid.velocity[1],
            DirectionY::Bottom => boid.position[1] -= boid.velocity[1],
        }
    }
}

fn view(_app: &App, _model: &Model, frame: Frame) {
    let draw = _app.draw();
    draw.background().color(WHITE);

    for boid in &_model._boids {
        let position = boid.position;
        draw.ellipse().xy(position).w_h(20.0, 20.0).color(BLACK);
    }

    draw.to_frame(_app, &frame).unwrap();
}

fn main() {
    nannou::app(model).update(update).view(view).run();
}
