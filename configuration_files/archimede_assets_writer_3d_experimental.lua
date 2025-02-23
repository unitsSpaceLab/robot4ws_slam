-- @author: Marco Giberna
-- @email: marco.giberna@studenti.units.it
-- @email: marcogiberna@gmail.com

VOXEL_SIZE = 5e-2

include "transform.lua"

options = {
  tracking_frame = "camera_accel_frame",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },
    {
      action = "dump_num_points",
    },
    {
       action = "intensity_to_color",
       min_intensity = 0.,
       max_intensity = 4095.,
    },

    -- Gray X-Rays. These only use geometry to color pixels.
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all",
      transform = YZ_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xy_all",
      transform = XY_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all",
      transform = XZ_TRANSFORM,
    },

    -- Now we recolor our points by frame and write another batch of X-Rays. It
    -- is visible in them what was seen by the horizontal and the vertical
    -- laser.
    {
      action = "color_points",
      frame_id = "horizontal_vlp16_link",
      color = { 255., 0., 0. },
    },
    {
      action = "color_points",
      frame_id = "vertical_vlp16_link",
      color = { 0., 255., 0. },
    },

    {
       action = "write_xray_image",
       voxel_size = VOXEL_SIZE,
       filename = "xray_yz_all_color",
       transform = YZ_TRANSFORM,
     },
     {
       action = "write_xray_image",
       voxel_size = VOXEL_SIZE,
       filename = "xray_xy_all_color",
       transform = XY_TRANSFORM,
     },
     {
       action = "write_xray_image",
       voxel_size = VOXEL_SIZE,
       filename = "xray_xz_all_color",
       transform = XZ_TRANSFORM,
     },
    
    -- generate the .ply and .pcd files
    {
      action = "write_ply",
      filename = "points.ply",
    }
    --{
    --  action = "write_pcd",
    --  filename = "points.pcd",
    --}
    -- {
    --   action = "write_probability_grid",
    --   filename = "probabilistic_grid.pcd",
    -- }
  }
}

return options