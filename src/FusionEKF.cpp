#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
				0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;

	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;

	/**
	TODO:
		* Finish initializing the FusionEKF.
		* Set the process and measurement noises
	*/
	// Initialize process Matrix and process noise


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_) 
	{
		/**
		TODO:
			* Initialize the state ekf_.x_ with the first measurement.
			* Create the covariance matrix.
			* Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		float px, py;
		float ro, theta;
		
		// initial measurement
		ekf_.x_ = VectorXd(4);

		// ekf_.P_ = Eigen::Identity(4, 4);
		ekf_.P_ << 0.1, 0.1, 0.1, 0.1,
				   0.1, 0.1, 0.1, 0.1,
				   0.1, 0.1, 0.1, 0.1,
				   0.1, 0.1, 0.1, 0.1;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
		{
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			ro = measurement_pack.raw_measurements_(0);
			theta = measurement_pack.raw_measurements_(1);

			px = ro * sin(theta);
			py = ro * cos(theta);

			ekf_.x_(0) = px ;
			ekf_.x_(1) = py ;
			// Initialize velocity with zero (arbitrary, but hopefully close enough guess)
			ekf_.x_(2) = 0.01f;
			ekf_.x_(3) = 0.01f;
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
		{
			/**
			Initialize state.
			*/
			px = measurement_pack.raw_measurements_(0);
			py = measurement_pack.raw_measurements_(1);

			ekf_.x_(0) = px;
			ekf_.x_(1) = py;
			// Initialize velocity with zero (arbitrary, but hopefully close enough guess)
			ekf_.x_(2) = 0.1f;
			ekf_.x_(3) = 0.1f;
		}

		previous_timestamp_ = measurement_pack.timestamp_;
		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	/**
	 TODO:
		 * Update the state transition matrix F according to the new elapsed time.
			- Time is measured in seconds.
		 * Update the process noise covariance matrix.
		 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	 */

	// Calculate the elapsed time here!  (Conversion into seconds)
	// Usually 0.05 seconds
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	// Process matrix
	ekf_.F_ << 1, 0, dt, 0,
			   0, 1, 0, dt,
			   0, 0, 1, 0,
			   0, 0, 0, 1;

	// Process noise
	float noise_ax, noise_ay;
	noise_ax = 9.0f;
	noise_ay = 9.0f;

	float dtPower2, dtPower3, dtPower4;
	dtPower2 = dt*dt;
	dtPower3 = dt*dt*dt;
	dtPower4 = dt*dt*dt*dt;

	ekf_.Q_ << dtPower4/4*noise_ax, 0, dtPower3/2*noise_ax, 0,
	           0, dtPower4/4*noise_ay, 0, dtPower3/2*noise_ay,
	           dtPower3/2*noise_ax, 0, dtPower2*noise_ax, 0,
	           0, dtPower3/2*noise_ay, 0, dtPower2*noise_ay;

	ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	/**
	 TODO:
		 * Use the sensor type to perform the update step.
		 * Update the state and covariance matrices.
	 */

	
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
	{	
		// Initialize Matrices before Update
		Hj_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.Init_Update(Hj_, R_radar_);

		// Radar updates
		VectorXd z = VectorXd(3);
		z(0) = measurement_pack.raw_measurements_(0);
		z(1) = measurement_pack.raw_measurements_(1);
		z(2) = measurement_pack.raw_measurements_(2);
		ekf_.UpdateEKF(z);	
	} 
	else 
	{
		// Initialize Matrices before Update
		ekf_.Init_Update(H_laser_, R_laser_);

		// Laser updates
		VectorXd z = VectorXd(2);
		z(0) = measurement_pack.raw_measurements_(0);
		z(1) = measurement_pack.raw_measurements_(1);
		ekf_.Update(z);
	}

	// print the output
	//cout << "x_ = " << ekf_.x_ << endl;
	//cout << "P_ = " << ekf_.P_ << endl;
}
