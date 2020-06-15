#include <MAX30100_beat_detector.h>
#include "stm32f4xx_it.h"

BeatDetectorState beat_state;
float threshold;
float beat_period;
float last_max_value;
uint32_t ts_last_beat;

void beat_detector() {
	beat_state = BEATDETECTOR_STATE_INIT;
	threshold = BEATDETECTOR_MIN_THRESHOLD;
	beat_period = 0;
	last_max_value = 0;
	ts_last_beat = 0;
}

int add_sample(float sample)
{
	return check_for_beat(sample);
}

float get_rate()
{
	if (beat_period != 0) {
		return 1 / beat_period * 1000 * 60;
	} else {
		return 0;
	}
}

float get_current_threshold()
{
	return threshold;
}

int check_for_beat(float sample)
{
	int beat_detected = 0;
	float delta;
	switch (beat_state) {
	case BEATDETECTOR_STATE_INIT:
		if (millis() > BEATDETECTOR_INIT_HOLDOFF) {
//                state = BEATDETECTOR_STATE_WAITING;   
			beat_state = BEATDETECTOR_STATE_WAITING;
		}
		break;

	case BEATDETECTOR_STATE_WAITING:
		if (sample > threshold) {
			threshold = min_check(sample, BEATDETECTOR_MAX_THRESHOLD);
//                state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;   
			beat_state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
		}

		// Tracking lost, resetting
		if (millis() - ts_last_beat > BEATDETECTOR_INVALID_READOUT_DELAY) {
			beat_period = 0;
			last_max_value = 0;
		}

		decrease_threshold();
		break;

	case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
		if (sample < threshold) {
//                state = BEATDETECTOR_STATE_MAYBE_DETECTED;   
			beat_state = BEATDETECTOR_STATE_MAYBE_DETECTED;
		} else {
			threshold = min_check(sample, BEATDETECTOR_MAX_THRESHOLD);
		}
		break;

	case BEATDETECTOR_STATE_MAYBE_DETECTED:
		if (sample + BEATDETECTOR_STEP_RESILIENCY < threshold) {
			// Found a beat
//                beatDetected = true;   
			beat_detected = 1;
			last_max_value = sample;
//                state = BEATDETECTOR_STATE_MASKING;   
			beat_state = BEATDETECTOR_STATE_MASKING;

			delta = millis() - ts_last_beat;

			if (delta) {
				beat_period = (float) BEATDETECTOR_BPFILTER_ALPHA * delta
						+ (1 - BEATDETECTOR_BPFILTER_ALPHA) * beat_period;
			}

			ts_last_beat = millis();
		} else {
//                state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;   
			beat_state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
		}
		break;

	case BEATDETECTOR_STATE_MASKING:
		if (millis() - ts_last_beat > BEATDETECTOR_MASKING_HOLDOFF) {
//                state = BEATDETECTOR_STATE_WAITING;   
			beat_state = BEATDETECTOR_STATE_WAITING;
		}
		decrease_threshold();
		break;
	}

	return beat_detected;
}

void decrease_threshold() //void BeatDetector::decreaseThreshold()
{
	// When a valid beat rate readout is present, target the
	if (last_max_value > 0 && beat_period > 0) {
		threshold -= last_max_value * (1 - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET)
				/ (beat_period / BEATDETECTOR_SAMPLES_PERIOD);
	} else {
		// Asymptotic decay
		threshold *= (float) BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
	}

	if (threshold < BEATDETECTOR_MIN_THRESHOLD) {
		threshold = BEATDETECTOR_MIN_THRESHOLD;
	}
}

int min_check(int a, int b) {
	return (a > b) ? b : a;
}   
   
