use plotters::prelude::*;

use pid_sim::{SecondOrder, Pid};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // ---- パラメータ ----
    let dt = 0.001;
    let t_end = 800.0;
    let r = 1.0; // ステップ目標値

    let wn = 2.0 / 100.0;
    let zeta: f64 = 2.0 / 100.0;

    let kp = 0.7;
    let ki = 0.01;
    let kd = 100.0;

    let mut plant = SecondOrder::new(wn, zeta); // 振動的なプラント
    let mut pid = Pid::new(kp, ki, kd);

    // ---- シミュレーション ----
    let n = (t_end / dt) as usize;
    let mut ts = Vec::with_capacity(n);
    let mut ys = Vec::with_capacity(n);
    let mut us = Vec::with_capacity(n);

    for i in 0..n {
        let t = i as f64 * dt;
        let u = pid.update(r, plant.y, dt);
        plant.step(u, dt);
        ts.push(t);
        ys.push(plant.y);
        us.push(u);
    }

    // ---- プロット ----
    let root = BitMapBackend::new("result.png", (1000, 800)).into_drawing_area();
    root.fill(&WHITE)?;
    let (upper, lower) = root.split_vertically(480);

    // let y_max = ys.iter().cloned().fold(f64::MIN, f64::max).max(r) * 1.1;
    let y_max = 1.5;
    let mut c1 = ChartBuilder::on(&upper)
        .caption("Step response (PID + 2nd-order plant)", ("sans-serif", 24))
        .margin(10)
        .x_label_area_size(35)
        .y_label_area_size(50)
        .build_cartesian_2d(0.0..t_end, 0.0..y_max)?;
    c1.configure_mesh().x_desc("t [s]").y_desc("y").draw()?;
    c1.draw_series(LineSeries::new(ts.iter().zip(&ys).map(|(&t, &y)| (t, y)), &BLUE))?
        .label("y")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));
    c1.draw_series(LineSeries::new(ts.iter().map(|&t| (t, r)), &RED))?
        .label("r")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));
    c1.configure_series_labels().border_style(&BLACK).draw()?;

    let u_min = us.iter().cloned().fold(f64::MAX, f64::min) - 0.5;
    let u_max = us.iter().cloned().fold(f64::MIN, f64::max) + 0.5;
    let mut c2 = ChartBuilder::on(&lower)
        .caption("Control input u", ("sans-serif", 24))
        .margin(10)
        .x_label_area_size(35)
        .y_label_area_size(50)
        .build_cartesian_2d(0.0..t_end, u_min..u_max)?;
    c2.configure_mesh().x_desc("t [s]").y_desc("u").draw()?;
    c2.draw_series(LineSeries::new(ts.iter().zip(&us).map(|(&t, &u)| (t, u)), &GREEN))?;

    root.present()?;
    println!("saved: pid_result.png");
    Ok(())
}