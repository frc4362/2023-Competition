package com.gemsrobotics.lib;


import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightResults {
	@JsonProperty("Results")
	public Results targetingResults;

	LimelightResults() {
		targetingResults = new Results();
	}
}