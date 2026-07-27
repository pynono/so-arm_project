from soarm_lab import arm

arm.live()
arm.go([0.25, 0.0, 15])
arm.go([0.20, 0.12, 10], grip=90)
arm.wait()
