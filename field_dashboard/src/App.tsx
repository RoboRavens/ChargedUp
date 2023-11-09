import {ThemeProvider, createTheme} from '@mui/material/styles';
import './App.css';
import { CssBaseline, Grid, Paper, Stack, styled } from '@mui/material';
import SportsEsportsOutlinedIcon from '@mui/icons-material/SportsEsportsOutlined';
import PowerOffIcon from '@mui/icons-material/PowerOff';
import PowerIcon from '@mui/icons-material/Power';
import React, { ReactElement } from 'react';
import { NetworkTables, NetworkTablesTopic, NetworkTablesTypeInfos } from 'ntcore-ts-client';
import { Adjust } from '@mui/icons-material';

const matchTimeFormatter = new Intl.NumberFormat('en-US', { maximumFractionDigits: 0 });

const darkTheme = createTheme({
  palette: {
    mode: 'dark',
  },
});

const Item = styled(Paper)(({ theme }) => ({
  backgroundColor: theme.palette.mode === 'dark' ? '#1A2027' : '#fff',
  ...theme.typography.body2,
  padding: theme.spacing(2),
  textAlign: 'center',
  color: theme.palette.text.secondary,
}));

const COLOR_GREEN = '#4caf50';
const COLOR_RED = '#b71c1c';

interface Topics {
  tabPub?: NetworkTablesTopic<string>;
  selectedYCoordinatePub?: NetworkTablesTopic<number>;
  selectedXCoordinatePub?: NetworkTablesTopic<number>;
}

const topics: Topics = {
  tabPub: undefined,
  selectedYCoordinatePub: undefined,
  selectedXCoordinatePub: undefined,
};

const NT_CORE = NetworkTables.getInstanceByURI("localhost");

const fieldWidth : number = 16.4846; // in meters
const fieldHeight : number = 8.1026; // in meters
var widthScale : number;
var heightScale : number;
var fieldImageBottomYCoordinate : number;
var fieldImageLeftXCoordinate : number;

class App extends React.Component<{}, {
  connected: boolean,
  alliance: string,
  driverStation: number,
  joystick0: boolean,
  joystick2: boolean,
  joystick3: boolean,
  matchTime: number,
  yCoordinateFromRobot: number,
  xCoordinateFromRobot: number,
  pixelYCoordinate: number,
  pixelXCoordinate: number,
  fieldSubzoneElements: ReactElement[],
}> {
  constructor(props: {}) {
    super(props);
    this.state = {
      connected: false,
      alliance: "NONE",
      driverStation: 0,
      joystick0: false,
      joystick2: false,
      joystick3: false,
      matchTime: -1,
      yCoordinateFromRobot: -1,
      xCoordinateFromRobot: -1,
      pixelYCoordinate: -1,
      pixelXCoordinate: -1,
      fieldSubzoneElements: [],
    };
  }

  setConnected(connected: boolean) {
    this.setState({
      connected: connected
    });
  }

  handleDriveStationChangeFromRobot(location: number | null) {
    this.setState({
      driverStation: location ?? 0
    });
  }

  handleJoystickStatusUpdateFromRobot(connected: boolean | null, index: number) {
    connected = connected ?? false;
    switch(index) {
      case 0:
        this.setState({joystick0: connected});
        break;
      case 2:
        this.setState({joystick2: connected});
        break;
      case 3:
        this.setState({joystick3: connected});
        break;
      default:
        break;
    }
  }

  setMatchTime(matchTime: number | null) {
    this.setState({
      matchTime: matchTime ?? -1
    });
  }

  setAlliance(alliance: string | null) {
    alliance = alliance ?? "NONE";
    this.setState({
      alliance: alliance
    })
  }

  setYCoordinateFromRobot(yCoordinate: number | null) {
    this.setState({
      yCoordinateFromRobot: yCoordinate ?? -1
    });
  }

  setXCoordinateFromRobot(xCoordinate: number | null) {
    this.setState({
      xCoordinateFromRobot: xCoordinate ?? -1
    });
  }

  drawFieldSubzones(fieldSubzones: number[] | null) {
    fieldSubzones = fieldSubzones ?? [];
    fieldSubzones.forEach(item => console.log(item))
    const fieldSubzoneElements : ReactElement[] = [];
    for (var i = 0; i < fieldSubzones.length; i+=4) {
      fieldSubzoneElements.push(<div style={{
        position: 'absolute', 
        left: fieldSubzones[i] * widthScale, 
        bottom: fieldSubzones[i+1] * heightScale,
        height: fieldSubzones[i+2] * heightScale,
        width: fieldSubzones[i+3] * widthScale,
        backgroundColor: '#FFFFFF00',
        border: '2px dashed white'
      }}></div>);
    }
    this.setState({
      fieldSubzoneElements: fieldSubzoneElements,
    });
  }

  setScaleVariables(e: {target: any}) {
    var rect = e.target.getBoundingClientRect();
    fieldImageLeftXCoordinate = rect.left;
    fieldImageBottomYCoordinate = rect.bottom;
    widthScale = (rect.right - rect.left) / fieldWidth;
    heightScale = (rect.bottom - rect.top) / fieldHeight;
  }

  componentDidMount() {
    NT_CORE.addRobotConnectionListener(this.setConnected.bind(this), true);
    
    topics.tabPub = NT_CORE.createTopic<string>('/ReactDash/Main/dpub/tab', NetworkTablesTypeInfos.kString);
    topics.tabPub.publish({retained: true});
    topics.tabPub.subscribe((value) => {console.log(value)}, true);

    NT_CORE.createTopic<number>('/ReactDash/Main/rpub/driverStation', NetworkTablesTypeInfos.kInteger)
    .subscribe((value) => { this.handleDriveStationChangeFromRobot(value); }, true);
    NT_CORE.createTopic<boolean>('/ReactDash/Main/rpub/joystick0', NetworkTablesTypeInfos.kBoolean)
    .subscribe((value) => { this.handleJoystickStatusUpdateFromRobot(value, 0); }, true);
    NT_CORE.createTopic<boolean>('/ReactDash/Main/rpub/joystick2', NetworkTablesTypeInfos.kBoolean)
    .subscribe((value) => { this.handleJoystickStatusUpdateFromRobot(value, 2); }, true);
    NT_CORE.createTopic<boolean>('/ReactDash/Main/rpub/joystick3', NetworkTablesTypeInfos.kBoolean)
    .subscribe((value) => { this.handleJoystickStatusUpdateFromRobot(value, 3); }, true);
    NT_CORE.createTopic<number>('/ReactDash/Autonomous/rpub/matchTime', NetworkTablesTypeInfos.kDouble)
    .subscribe((value) => { this.setMatchTime(value); }, true);
    NT_CORE.createTopic<string>('/ReactDash/Autonomous/rpub/alliance', NetworkTablesTypeInfos.kString)
    .subscribe((value) => { this.setAlliance(value); }, true);
    NT_CORE.createTopic<number[]>('/ReactDash/Teleop/rpub/fieldZones', NetworkTablesTypeInfos.kDoubleArray)
    .subscribe((value) => { this.drawFieldSubzones(value); }, true);

    topics.selectedYCoordinatePub = NT_CORE.createTopic<number>('/ReactDash/Teleop/dpub/selectedYCoordinate', NetworkTablesTypeInfos.kDouble);
    topics.selectedYCoordinatePub.publish({retained: true});
    topics.selectedXCoordinatePub = NT_CORE.createTopic<number>('/ReactDash/Teleop/dpub/selectedXCoordinate', NetworkTablesTypeInfos.kDouble);
    topics.selectedXCoordinatePub.publish({retained: true});

    NT_CORE.createTopic<number>('/ReactDash/Teleop/rpub/selectedYCoordinate', NetworkTablesTypeInfos.kDouble, -1)
    .subscribe((value) => { this.setYCoordinateFromRobot(value); }, true);
    NT_CORE.createTopic<number>('/ReactDash/Teleop/rpub/selectedXCoordinate', NetworkTablesTypeInfos.kDouble, -1)
    .subscribe((value) => { this.setXCoordinateFromRobot(value); }, true);

  }

  scaleAndSendCoordinates(e: { clientX: any; clientY: any}) {
    var x = e.clientX - fieldImageLeftXCoordinate;
    var y = fieldImageBottomYCoordinate - e.clientY;
    this.setState({
      pixelXCoordinate: x,
      pixelYCoordinate: y,
    });
    // Scale the given coordinates to meters
    x /= widthScale;
    y /= heightScale;
    topics.selectedXCoordinatePub?.setValue(x);
    topics.selectedYCoordinatePub?.setValue(y);
  }

  render() {
    let allianceColor = '#808080'
      switch(this.state.alliance) {
        case "Blue":
          allianceColor = '#1565c0'
          break;
        case "Red":
          allianceColor = '#ff1744'
          break;
        default:
      }
    return (
      <ThemeProvider theme={darkTheme}>
        <CssBaseline/>
        <Grid container component="main" sx={{minHeight: '100%', padding: darkTheme.spacing(2), paddingTop: darkTheme.spacing(2) }} columns={{ xs: 5, sm: 5, md: 5 }} spacing={2}>
          <Grid item xs={5} sm={5} md={5}>
            <Stack direction="row" alignItems="stretch" justifyContent="space-evenly" spacing={2}>
              <Item sx={{width: '100%', fontSize: '2rem', backgroundColor: allianceColor}}>
                {this.state.alliance} Alliance
              </Item>
              <Item sx={{width: '100%', fontSize: '2rem'}}>
                  Driver Station: {this.state.driverStation}
              </Item>
              <Item sx={{padding: '0px', width: '100%'}}>
                <SportsEsportsOutlinedIcon sx={{fontSize: '5rem', color: this.state.joystick0 ? COLOR_GREEN : COLOR_RED}} />
                <SportsEsportsOutlinedIcon sx={{fontSize: '5rem', color: this.state.joystick2 ? COLOR_GREEN : COLOR_RED}} />
                <SportsEsportsOutlinedIcon sx={{fontSize: '5rem', color: this.state.joystick3 ? COLOR_GREEN : COLOR_RED}} />
              </Item>
              <Item>{this.state.connected ? <PowerIcon sx={{fontSize: '3rem', color: COLOR_GREEN}} /> : <PowerOffIcon sx={{fontSize: '3rem', color: COLOR_RED}} />}</Item>
              <Item sx={{fontSize: '3rem', width: '100%', padding: darkTheme.spacing(1)}}>
                {this.state.connected ? matchTimeFormatter.format(this.state.matchTime) : (<span style={{color: COLOR_RED}}>disconnected!</span>)}
              </Item>
            </Stack>
          </Grid>
          <Grid item xs={5} sm={5} md={5}>
            <Stack direction='row' spacing={2}>
              <Item>
                <div style={{position: 'relative'}}>
                  <img src='./2023-field-cropped.png' style={{position: 'relative', height: '70vh'}} onClick={this.scaleAndSendCoordinates.bind(this)} onLoad={this.setScaleVariables.bind(this)}></img>
                  <Adjust style={{position: 'absolute', bottom: this.state.pixelYCoordinate, left: this.state.pixelXCoordinate, transform: 'translate(-50%, 50%)'}}/>
                  <div onClick={this.scaleAndSendCoordinates.bind(this)}>{this.state.fieldSubzoneElements}</div>
                </div>
              </Item>
              <Stack><Item>Robot Received Coordinates: {this.state.xCoordinateFromRobot}, {this.state.yCoordinateFromRobot}</Item></Stack>
            </Stack>
          </Grid>
        </Grid>
      </ThemeProvider>
    );
  }
}

export default App;
