# Field Display Dashboard

This dashboard may be used in future seasons to give drivers the ability to quickly specify a target coordinate on the field, which can be easily manipulated in the robot code

## How to Repurpose this Code for the Season

### Replacing the field image

To display a new field, first upload the image to the `public` directory. Make sure the image is cropped so that *only* the inside of the field is in the image. For example, you may have to cut out the driver station portion of the field, which is technically outside of the field. This ensures that the image coordinates are scaled properly.

Next, go to line 251 of `App.tsx` where you'll see the line `<img src='./2023-field-cropped.png'...`. Replace the image name with that of the image you just uploaded to the `public` directory.

Now your new field image will display! When you click on the image, the code will automatically scale the coordinates using the physical field dimensions and send those values to the robot code. When the robot recieves those values, they will be displayed on the righthand side of the dashboard.

### Displaying field zones

Since field zones are created and handled primarily in the robot code, the dashboard utilizes values from those FieldSubzone objects to display field zones. To create new field zones, please see documentation for that portion of the robot codebase (non-existent rn). On line 78 of `subsystems/TabletScoring/TeleopSubsystem.java`, replace the argument in `getFieldZonesAsDashboardData()` with the list of FieldSubzone objects that you want to send to the dashboard. After updating this, you should see all of the zones display with white dashed lines as borders.