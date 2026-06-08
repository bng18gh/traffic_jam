# traffic_jam
EE470 Final Project Repository


This repository contains three files: two for visual evaluation and one for numerical evaluation. 

## baseline_three_to_two_merge_control_match.m
Visual depiction of the baseline merging algorithm. Setup includes three lanes and three vehicles. The farmost right lane disappears, and so the rightmost car needs to merge into the middle lane.  
The baseline algorithm uses right of way to decide which cars move ahead and which slow down relative to their neighbors. Simply run the program to obersve the merging vehicle behavior. A "good" run is indicated by the merging vehicle completing its merge by the match line without colliding with another vehicle.

#### Consider tuning these variables to alter the merging performance:
yieldSpeedMargin: When giving right of way, how much slower the yielding car will be. Important metric for faster cars passing slower ones. However, too slow a deviation might slow down the merging car even further.  
mergeSafetyHorizon: How far away a car must be in order for the surrounding area to be considered safe. Larger horizons delay the merge and in turn means the merging vehicle slows down more.  
unsafeYieldSpeed: Target speed for merging vehicle if not safe to merge, implemented to extend the distance gap. Slower yield speeds extend the gap faster, but be aware the vehicle must accelerate from that yield speed back up to nominal as it merges in, which takes longer if the yield speed is lower.  


## cbf_three_to_two_merge_supervised.m
hi there again
## run_merge_controller_metrics.m
sup
