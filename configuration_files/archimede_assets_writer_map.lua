-- @author: Marco Giberna
-- @email: marco.giberna@studenti.units.it
-- @email: marcogiberna@gmail.com

options = {
    tracking_frame = "Archimede_base_link",
    pipeline = {
      {
        action = "min_max_range_filter",
        min_range = 1.,
        max_range = 60.,
      },
      {
        action = "write_ros_map",
        range_data_inserter = {
          insert_free_space = true,
          hit_probability = 0.55,
          miss_probability = 0.49,
        },
        filestem = "map",
        resolution = 0.05,
      }
    }
  }
  
  return options