use ndarray::*;

fn main() {
  println!("Hello, world!");

  let mut theta_rwl: f64 = 0.0;  // Right Waist Roll
  let mut theta_rwp: f64 = 0.0;  // Right Waist Pitch
  let mut theta_rkp: f64 = 0.0;  // Right Knee Pitch
  let mut _theta_lwl: f64 = 0.0;  // Left Waist Roll
  let mut _theta_lwp: f64 = 0.0;  // Left Waist Pitch
  let mut _theta_lkp: f64 = 0.0;  // Left Knee Pitch

  let vp_c_rwl: Array1<f64> = array![-0.091, -0.075, 0.];
  let vp_rwl_rwp: Array1<f64> = array![0.091, -0.055, 0.];
  let vp_rwp_rkp: Array1<f64> = array![0., 0., -0.16];
  let vp_rkp_re: Array1<f64> = array![0., 0., -0.173];

  let vp_c_re: Array1<f64> = array![0.0, -0.075 + -0.055, -(0.16_f64.powf(2.) + 0.173_f64.powf(2.)).sqrt()];  // target position
  println!("vp_c_re: {}", &vp_c_re);

// IK
  let el1: i32 = ((&vp_c_re[1] - &vp_c_rwl[1]) * 1000.0) as i32;
  let el2: f64 = el1 as f64 / 1000.;  // 丸め誤差だかが原因でacos()がnanになるので、i32に変換してからfloatに戻す
  theta_rwl = (el2 / &vp_rwl_rwp[1]).acos();
  println!("theta_rwl: {}", &theta_rwl * 180. / std::f64::consts::PI);

  let rm_rwl: Array2<f64> = array![
    [1., 0., 0.,],
    [0., theta_rwl.cos(), -theta_rwl.sin()],
    [0., theta_rwl.sin(), theta_rwl.cos()]
  ];
  let vp_c_rwp: Array1<f64> = &vp_c_rwl + &rm_rwl.dot(&vp_rwl_rwp);
  //println!("vp_c_rwp: {}", &vp_c_rwp);
  let vr: Array1<f64> = &vp_c_re - &vp_c_rwp;
  //println!("vr: {}", &vr);
  let a: f64 = ((&vr[0].powf(2.) + &vr[1].powf(2.)).sqrt() + &vr[2].powf(2.)).sqrt();
  //println!("a: {}", &a);  // ここまでOK

  let _el3 = (-&a.powf(2.) + &vp_rkp_re[2].abs().powf(2.) + &vp_rwp_rkp[2].abs().powf(2.)) / (2. * &vp_rkp_re[2].abs() * &vp_rwp_rkp[2].abs());
  //println!("el3: {}", &el3);
  let _el4 = -&a.powf(2.) + &vp_rkp_re[2].abs().powf(2.) + &vp_rwp_rkp[2].abs().powf(2.);
  //println!("el4: {} ({} {} {})", &el4, -a.powf(2.), &vp_rkp_re[2].abs().powf(2.), &vp_rwp_rkp[2].abs().powf(2.));
  theta_rkp = ((-&a.powf(2.) + &vp_rkp_re[2].powf(2.) + &vp_rwp_rkp[2].powf(2.)) / (2. * &vp_rkp_re[2].abs() * &vp_rwp_rkp[2].abs())).acos();

  theta_rwp = ((-&vp_rkp_re[2].powf(2.) + &a.powf(2.) + &vp_rwp_rkp[2].powf(2.)) / (2. * &a.abs() * &vp_rwp_rkp[2].abs())).acos();
  println!("theta_rwp: {}", &theta_rwp * 180. / std::f64::consts::PI);

  println!("theta_rkp: {}", &theta_rkp * 180. / std::f64::consts::PI);

// FK
  let rm_rwp: Array2<f64> = array![
    [-theta_rwp.cos(), 0., -theta_rwp.sin()],
    [0., 1., 0.],
    [theta_rwp.sin(), 0., -theta_rwp.cos()]
  ];
  let rm_rkp: Array2<f64> = array![
    [theta_rkp.cos(), 0., theta_rkp.sin()],
    [0., 1., 0.],
    [-theta_rkp.sin(), 0., theta_rkp.cos()]
  ];

}
