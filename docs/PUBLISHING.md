# GitHub publishing checklist

Use this checklist before making the GNU campus simulator public.

1. Confirm redistribution permission for measured campus data, vehicle assets,
   textures, logos, and every third-party dependency.
2. Install Git LFS on the host, then run `git lfs install` before staging the
   Stage D runtime USD files.
3. Verify `git lfs ls-files` includes every file under
   `assets/maps/gnu_campus_stage_D_full_usd/usd/components/`.
4. Confirm raw PCD data and Stage B/C intermediates remain ignored.
5. Review `git status`, staged file sizes, and the staged diff. Do not include
   logs, caches, credentials, machine-specific paths, or generated source data.
6. Run the unit tests and launch the persistent GUI using the documented command.
7. Verify the map, vehicle, HUD, ROS topics, 0.5 m/s drive/stop behavior, and the
   60 km/h target-speed clamp.
8. Commit locally. Push only after selecting the intended public/private GitHub
   visibility and checking the repository's Git LFS storage/bandwidth limits.

No GitHub push is performed automatically by this project setup.
