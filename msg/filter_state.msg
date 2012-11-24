# constants
uint32 PTAM_IDLE = 0           # PTAM not running.
uint32 PTAM_INITIALIZING = 1   # initialization (trails)
uint32 PTAM_LOST = 2           # ptam is running, but lost
uint32 PTAM_GOOD = 3           # tracking quality OK
uint32 PTAM_BEST = 4           # tracking quality best
uint32 PTAM_TOOKKF = 5         # just took a new KF (equivalent to PTAM_BEST)
uint32 PTAM_FALSEPOSITIVE = 6  # ptam thinks it is good, but its estimate is rejected.

# header
Header      header

# ----------------- raw 10d filter state ----------------------------
float32     x
float32     y
float32     z
float32     dx
float32     dy
float32     dz
float32     roll
float32     pitch
float32     yaw
float32     dyaw



# --------------------- other values ---------------------------
float32     scale         # ptam scale factor (PTAMpos * scale = WORLDpos).
uint32      ptamState
float32     scaleAccuracy # if scale is very inaccurate, this is about 0.5, and grows up to 1 (=very accurate).


# ----------------- propagated from drone messages: -----------------
# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
uint32       droneState
float32      batteryPercent    # 0 means no battery, 100 means full battery
