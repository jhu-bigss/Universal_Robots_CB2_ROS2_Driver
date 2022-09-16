def SimpleServer():
#BIGSSURServer
#

  # setup socket to receive client commands
  ADDRESS = "${HOSTNAME}"
  PORT = ${HOSTPORT}
  prog_running = socket_open(ADDRESS, PORT)
  if prog_running:
    textmsg("Successfully connect to server: ", ADDRESS)
  else:
    textmsg("Could not connect to server: ")
    textmsg(ADDRESS, PORT)
  end

  # using prog_running instead of while True eliminates the infinite loop detected
  # UR Script runtime error, and is slightly more graceful when terminating threads


  # setup the spatial velocity thread
  cur_spatial_vel_valid = False
  cur_spatial_vel = [0,0,0,0,0,0]
  thread move_spatial_vel():
    while prog_running:
      if cur_spatial_vel_valid:
        speedl(cur_spatial_vel, 0.15, 0)
      else:
        sync()
      end
    end
  end

  # start the spatial velocity thread
  spatial_vel_thrd = run move_spatial_vel()


  # setup the joint velocity thread
  cur_joint_vel_valid = False
  cur_joint_vel = [0,0,0,0,0,0]
  thread move_joint_vel():
    while prog_running:
      if cur_joint_vel_valid:
        speedj(cur_joint_vel, 1.0, 0)
      else:
        sync()
      end
    end
  end

  # start the joint velocity thread
  move_joint_vel_thrd = run move_joint_vel()


  max_vel_mult = 1.0 # user-settable multiplier for the speed. [0 - 1]

  # Main run loop
  while prog_running:

    val = socket_read_ascii_float(7)
    mode_val = val[1]
    textmsg("num_of_var received: ", val[0])
    textmsg("mode = ", mode_val)

    if val[0] >= 7:
      if mode_val == 0:
        # spatial position move
        textmsg("spatial position move")
        cur_spatial_vel_valid = False
        cur_joint_vel_valid = False

        movel([val[2],
                val[3],
                val[4],
                val[5],
                val[6],
                val[7]],
                1.2,
                max_vel_mult*0.25)

      elif mode_val == 1:
        # joint position move
        textmsg("joint position move")
        cur_spatial_vel_valid = False
        cur_joint_vel_valid = False

        movej([val[2],
               val[3],
               val[4],
               val[5],
               val[6],
               val[7]],
               1.4,
               max_vel_mult*1.05)

      elif mode_val == 2:
        # spatial velocity move
        textmsg("spatial velocity commanded")
        cur_joint_vel_valid = False

        cur_spatial_vel = [val[2],
                           val[3],
                           val[4],
                           val[5],
                           val[6],
                           val[7]]
        cur_spatial_vel_valid = True

      elif mode_val == 3:
        # joint velocity move
        textmsg("joint velocity commanded")
        cur_spatial_vel_valid = False

        cur_joint_vel = [val[2],
                         val[3],
                         val[4],
                         val[5],
                         val[6],
                         val[7]]
        cur_joint_vel_valid = True

      elif mode_val == 4:
        # stop linear in joint space
        textmsg("stopj")
        cur_spatial_vel_valid = False
        cur_joint_vel_valid = False

        stopj(val[2])
      end

    elif val[0] >= 2:

      if mode_val == 5:
        # Set the movej speed
        max_vel_mult = val[2]
        textmsg("speed percent set to", max_vel_mult)
      end

    end

    if mode_val > 9:
      # invalid mode!
      cur_spatial_vel_valid = False
      cur_joint_vel_valid = False
      textmsg("Invalid Mode: ", mode_val)
    end

    sync()

  end

  join spatial_vel_thrd
  join move_joint_vel_thrd

  socket_close()

end

