import React from 'react';
import './App.css';
import { ThemeProvider, createTheme } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import { NetworkTables, NetworkTablesTopic, NetworkTablesTypeInfos } from 'ntcore-ts-client';

import ListItemButton from '@mui/material/ListItemButton';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';
import Divider from '@mui/material/Divider';
import ListSubheader from '@mui/material/ListSubheader';

import SmartToyOutlinedIcon from '@mui/icons-material/SmartToyOutlined';
import SportsEsportsOutlinedIcon from '@mui/icons-material/SportsEsportsOutlined';
import CarRepairOutlinedIcon from '@mui/icons-material/CarRepairOutlined';
import PrecisionManufacturingOutlinedIcon from '@mui/icons-material/PrecisionManufacturingOutlined';
import WarningIcon from '@mui/icons-material/Warning';
import CheckBoxIcon from '@mui/icons-material/CheckBox';
import AccountBoxIcon from '@mui/icons-material/AccountBox';
import RefreshIcon from '@mui/icons-material/Refresh';
import PowerIcon from '@mui/icons-material/Power';
import PowerOffIcon from '@mui/icons-material/PowerOff';

import Grid from '@mui/material/Grid';
import Paper from '@mui/material/Paper';

import { experimentalStyled as styled } from '@mui/material/styles';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import Radio from '@mui/material/Radio';
import TextField from '@mui/material/TextField';
import Stack from '@mui/material/Stack';

const matchTimeFormatter = new Intl.NumberFormat('en-US', { maximumFractionDigits: 0 });

const darkTheme = createTheme({
  palette: {
    mode: 'dark',
  },
});

const formatNumber = (number: number | null): string | null => {
  if (number == null) {
    return null;
  }

  return (new Intl.NumberFormat('en-US', {
    maximumFractionDigits: 0
  }).format(number));
};

const columnName = ['A','B','C','D','E','F','G','H','I'];
const columnIndexToName = (column: number) => {
  if (column < 0) {
    return "NONE";
  }

  return columnName[column];
}

const rowName = ['3', '2', '1'];
const rowIndexToName = (row: number) => {
  if (row < 0) {
    return "NONE";
  }

  return rowName[row];
}

const Item = styled(Paper)(({ theme }) => ({
  backgroundColor: theme.palette.mode === 'dark' ? '#1A2027' : '#fff',
  ...theme.typography.body2,
  padding: theme.spacing(2),
  textAlign: 'center',
  color: theme.palette.text.secondary,
}));

const COLOR_GREEN = '#4caf50';
const COLOR_RED = '#b71c1c';

const CONE_HIGHLIGHT = '#fbcc57';
const CUBE_HIGHLIGHT = '#8561c5';
const OTHER_HIGLIGHT = '#35afea';

const renderBigCone = (selected: boolean) => {
  let sx = selected ? {backgroundColor:CONE_HIGHLIGHT} : {};
  return <Item sx={sx}><img src="./cone-icon.png" style={{maxHeight: '140px'}} /></Item>
}

const renderBigCube = (selected: boolean) => {
  let sx = selected ? {backgroundColor:CUBE_HIGHLIGHT} : {};
  return <Item sx={sx}><img src="./cube-icon.png" style={{maxHeight: '140px'}} /></Item>
}

const renderSubstation = (substation: string, selectedSubstation: string, handleClick: (clickedSubstation: string) => void) => {
  let sx = substation === selectedSubstation ? {backgroundColor:OTHER_HIGLIGHT} : {};
  return <Item sx={sx} onClick={() => handleClick(substation)}>{substation}</Item>
}

const getIcon = (column: number, row: number, selected: boolean, clickHandler: () => void): React.ReactElement => {
  var name = columnIndexToName(column) + rowIndexToName(row);
  var nameElement = <div style={{position: 'absolute', fontSize: '20px'}}>{name}</div>;
  if (row == 2) {
    let any = <img src="./any-icon.png" style={{maxHeight: '70px'}} />;
    return selected
    ? <Item onClick={clickHandler} sx={{backgroundColor:OTHER_HIGLIGHT}}>{nameElement}{any}</Item>
    : <Item onClick={clickHandler}>{nameElement}{any}</Item>;
  }

  switch (column) {
    case 1:
    case 4:
    case 7:
      let cube = <img src="./cube-icon.png" style={{maxHeight: '70px'}} />;
      return selected
        ? <Item onClick={clickHandler} sx={{backgroundColor:CUBE_HIGHLIGHT}}>{nameElement}{cube}</Item>
        : <Item onClick={clickHandler}>{nameElement}{cube}</Item>;
    default:
      let cone = <img src="./cone-icon.png" style={{maxHeight: '70px'}} />;
      return selected
        ? <Item onClick={clickHandler} sx={{backgroundColor:CONE_HIGHLIGHT}}>{nameElement}{cone}</Item>
        : <Item onClick={clickHandler}>{nameElement}{cone}</Item>;
  }
}

const columnOrRowParseInt = (value: string | null): number => {
  let num = parseInt(value ?? "-1");
  if (Number.isNaN(num)) {
    num = -1;
  }

  return num;
}

interface Topics {
  tabPub?: NetworkTablesTopic<string>;
  selectedAutoPub?: NetworkTablesTopic<string>;
  selectedColumnPub?: NetworkTablesTopic<string>;
  selectedRowPub?: NetworkTablesTopic<string>;
  selectedShapePub?: NetworkTablesTopic<string>;
  selectedSubstationPub?: NetworkTablesTopic<string>;
}

const topics: Topics = {
  tabPub: undefined,
  selectedAutoPub: undefined,
  selectedColumnPub: undefined,
  selectedRowPub: undefined,
  selectedShapePub: undefined,
  selectedSubstationPub: undefined,
};

// const NT_CORE = NetworkTables.getInstanceByTeam(1188);
const NT_CORE = NetworkTables.getInstanceByURI("localhost");

class App extends React.Component<{}, {
  connected: boolean,
  selectedTab: string,
  driverStation: number,
  joystick0: boolean,
  joystick2: boolean,
  joystick3: boolean,
  matchTime: number,
  alliance: string,
  selectedAuto: string,
  selectedAutoFromRobot: string,
  autoOptions: Array<string>,
  selectedColumnFromRobot: number,
  selectedRowFromRobot: number,
  selectedShapeFromRobot: string,
  selectedSubstationFromRobot: string,
}> {
  constructor(props: {}) {
    super(props);
    this.state = {
      connected: false,
      selectedTab: "Autonomous",
      driverStation: 0,
      joystick0: false,
      joystick2: false,
      joystick3: false,
      matchTime: -1,
      alliance: "NONE",
      selectedAuto: "NONE",
      selectedAutoFromRobot: "NONE",
      autoOptions: [],
      selectedColumnFromRobot: -1,
      selectedRowFromRobot: -1,
      selectedShapeFromRobot: "NONE",
      selectedSubstationFromRobot: "NONE",
    };
  }

  setConnected(connected: boolean) {
    this.setState({
      connected: connected
    });
  }

  handleTabClick(tab: string) {
    console.log("handleTabClick: " + tab);
    this.setState({
      selectedTab: tab
    });

    topics.tabPub?.setValue(tab);
  }

  initTab(tab: string | null) {
    console.log("initTab: " + tab);
    this.setState({
      selectedTab: tab ?? "Autonomous"
    });
  }

  robotSendTab(tab: string | null) {
    console.log("robotSendTab: " + tab);
    this.setState({
      selectedTab: tab ?? "Autonomous"
    });
  }

  handleDriveStationChangeFromRobot(location: number | null) {
    this.setState({
      driverStation: location ?? 0
    });
  }

  handleJoystickStatusUpdateFromRobot(connected: boolean | null, index: number) {
    console.log("handleJoystickStatusUpdateFromRobot: " + connected + "-" + index);
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

  onOptionsChange(options: string[] | null) {
    this.setState({
      autoOptions: options ?? [""]
    })
  }

  handleRadioChange(event: React.ChangeEvent<HTMLInputElement>) {
    let value = (event.target as HTMLInputElement).value;
    console.log("handleRadioChange: " + value);
    this.setState({
      selectedAuto: value
    });

    topics.selectedAutoPub?.setValue(value);
  };

  setSelectedAuto(selected: string | null) {
    this.setState({
      selectedAuto: selected ?? "NONE"
    });
  }

  setSelectedAutoFromRobot(selected: string | null) {
    console.log("setSelectedAutoFromRobot: " + selected);
    let auto = selected ?? "None Selected...";
    this.setState({
      selectedAutoFromRobot: auto
    });

    this.setState({
      selectedAuto: auto
    });
  }

  onScoringLocationClick(column: number, row: number) {
    console.log("onScoringLocationClick: " + column + "|" + row);
    if (this.state.selectedColumnFromRobot === column && this.state.selectedRowFromRobot === row) {
      column = -1;
      row = -1;
    }

    topics.selectedColumnPub?.setValue("" + column);
    topics.selectedRowPub?.setValue("" + row);
  }

  onScoringPieceClick(piece: string) {
    console.log("onScoringPieceClick: " + piece);
    if (this.state.selectedShapeFromRobot === piece) {
      piece = "NONE";
    }

    topics.selectedShapePub?.setValue(piece);
  }

  onSubstationClick(substation: string) {
    if (this.state.selectedSubstationFromRobot === substation) {
      substation = "NONE";
    }

    topics.selectedSubstationPub?.setValue(substation);
  }

  setColumnFromRobot(value: string | null) {
    console.log("setColumnFromRobot: " + value);
    value = value ?? "-1"
    this.setState({
      selectedColumnFromRobot: columnOrRowParseInt(value)
    });

    topics.selectedColumnPub?.setValue(value);
  }

  setRowFromRobot(value: string | null){
    console.log("setRowFromRobot: " + value);
    value = value ?? "-1"
    this.setState({
      selectedRowFromRobot: columnOrRowParseInt(value)
    });

    topics.selectedRowPub?.setValue(value);
  }

  setScoringShapeFromRobot(value: string | null) {
    value = value ?? "NONE"
    this.setState({
      selectedShapeFromRobot: value
    });

    topics.selectedShapePub?.setValue(value);
  }

  setSubstationFromRobot(value: string | null) {
    this.setState({
      selectedSubstationFromRobot: value ?? "NONE"
    });
  }

  resetDashboard() {
    NT_CORE.client.cleanup();
    window.location.reload();
  }

  componentDidMount() {
    NT_CORE.addRobotConnectionListener(this.setConnected.bind(this), true);

    topics.tabPub = NT_CORE.createTopic<string>('/ReactDash/Main/dpub/tab', NetworkTablesTypeInfos.kString);
    topics.tabPub.publish({retained: true});
    topics.tabPub.subscribe((value) => { this.initTab(value); }, true);

    NT_CORE.createTopic<string>('/ReactDash/Main/rpub/goTotab', NetworkTablesTypeInfos.kString)
    .subscribe((value) => { this.robotSendTab(value); }, true);

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

    const optionsTopic = NT_CORE.createTopic<string[]>('/ReactDash/Autonomous/rpub/options', NetworkTablesTypeInfos.kString);
    optionsTopic.subscribe((value) => { this.onOptionsChange(value); }, true);
    topics.selectedAutoPub = NT_CORE.createTopic<string>('/ReactDash/Autonomous/dpub/selectedAuto', NetworkTablesTypeInfos.kString, "No Auto");
    topics.selectedAutoPub.publish({retained: true}); // make us the publisher for this topic and tell the server retain the value if we disconnect

    NT_CORE.createTopic<string>('/ReactDash/Autonomous/rpub/selectedAuto', NetworkTablesTypeInfos.kString, "No Auto")
      .subscribe((value) => { this.setSelectedAutoFromRobot(value); }, true);

    topics.selectedColumnPub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/column', NetworkTablesTypeInfos.kString);
    topics.selectedColumnPub.publish({retained: true});
    topics.selectedRowPub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/row', NetworkTablesTypeInfos.kString);
    topics.selectedRowPub.publish({retained: true});
    topics.selectedShapePub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/shape', NetworkTablesTypeInfos.kString, "");
    topics.selectedShapePub.publish({retained: true});
    topics.selectedSubstationPub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/substation', NetworkTablesTypeInfos.kString, "");
    topics.selectedSubstationPub.publish({retained: true});

    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/column', NetworkTablesTypeInfos.kString, "-1")
    .subscribe((value) => { this.setColumnFromRobot(value); }, true);
    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/row', NetworkTablesTypeInfos.kString, "-1")
    .subscribe((value) => { this.setRowFromRobot(value); }, true);
    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/shape', NetworkTablesTypeInfos.kString, "NONE")
    .subscribe((value) => { this.setScoringShapeFromRobot(value); }, true);
    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/substation', NetworkTablesTypeInfos.kString, "NONE")
    .subscribe((value) => { this.setSubstationFromRobot(value); }, true);
  }

  componentWillUnmount() {
    NT_CORE.client.cleanup();
    console.log("componentWillUnmount");
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
        <CssBaseline />
        <Grid container component="main" sx={{ height: '100vh', paddingRight: darkTheme.spacing(2), paddingTop: darkTheme.spacing(2) }} columns={{ xs: 12, sm: 12, md: 12 }} spacing={2}>
          <Grid item  xs={2} sm={2} md={2}>
            <ListItemButton selected={this.state.selectedTab == "Autonomous"} onClick={() => this.handleTabClick("Autonomous")}>
              <ListItemIcon>
                <SmartToyOutlinedIcon />
              </ListItemIcon>
              <ListItemText primary="Autonomous" />
            </ListItemButton>
            <Divider sx={{ my: 1 }} />
            <ListItemButton selected={this.state.selectedTab == "Teleop"} onClick={() => this.handleTabClick("Teleop")}>
              <ListItemIcon>
                <SportsEsportsOutlinedIcon />
              </ListItemIcon>
              <ListItemText primary="Teleop" />
            </ListItemButton>
            <Divider sx={{ my: 1 }} />
            <ListSubheader component="div" inset>
              Diagnostics
            </ListSubheader>
            <ListItemButton>
              <ListItemIcon>
                <CarRepairOutlinedIcon />
              </ListItemIcon>
              <ListItemText primary="DriveTrain" />
            </ListItemButton>
            <ListItemButton>
              <ListItemIcon>
                <PrecisionManufacturingOutlinedIcon />
              </ListItemIcon>
              <ListItemText primary="Arm" />
            </ListItemButton>
            <ListItemButton onClick={() => this.resetDashboard()}>
              <ListItemIcon>
                <RefreshIcon />
              </ListItemIcon>
              <ListItemText primary="Refresh" />
            </ListItemButton>
          </Grid>
          <Grid item xs={10} sm={10} md={10}>
            <Grid container columns={{ xs: 9, sm: 9, md: 9 }} spacing={2}>
              <Grid item xs={9} sm={9} md={9}>
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
              {this.state.selectedTab == "Autonomous" &&
                <React.Fragment>
                  <Grid item xs={5} sm={5} md={5}>
                    <RadioGroup
                      value={this.state.selectedAuto}
                      onChange={this.handleRadioChange.bind(this)}
                    >
                      {this.state.autoOptions.map((option, _) => (
                        <FormControlLabel value={option} control={<Radio />} label={option} componentsProps={{typography: {fontSize: '1.3rem'}}} />
                      ))}
                    </RadioGroup>
                  </Grid>
                  <Grid item xs={4} sm={4} md={4}>
                    <TextField
                      fullWidth
                      disabled
                      label="Dashboard Selected Autonomous"
                      value={this.state.selectedAuto}
                      sx={{marginBottom: darkTheme.spacing(2)}}
                      InputProps={{sx:{fontSize: '2rem'}}}
                    />
                    {this.state.selectedAuto === this.state.selectedAutoFromRobot
                      ? <CheckBoxIcon sx={{fontSize: 50, color: COLOR_GREEN}} />
                      : <WarningIcon sx={{fontSize: 50, color: COLOR_RED}} />
                    }
                    <TextField
                      fullWidth
                      disabled
                      label="Robot Selected Autonomous"
                      value={this.state.selectedAutoFromRobot}
                      sx={{marginTop: darkTheme.spacing(2)}}
                      InputProps={{sx:{fontSize: '2rem'}}}
                    />
                  </Grid>
                </React.Fragment>
              }
              {this.state.selectedTab == "Teleop" &&
                <React.Fragment>
                  {Array.from(Array(9)).map((_, column) => (
                    <React.Fragment>
                      <Grid item xs={1} sm={1} md={1} key={column} sx={{textAlign: 'center', fontSize: 30}}>
                        {columnIndexToName(column)}
                      </Grid>
                    </React.Fragment>
                  ))}
                  {Array.from(Array(3)).map((_, row) => (
                    <React.Fragment>
                    {Array.from(Array(9)).map((_, column) => (
                      <Grid item xs={1} sm={1} md={1} key={(row + "-" + column)}>
                        {getIcon(column, row, this.state.selectedColumnFromRobot === column && this.state.selectedRowFromRobot === row, () => this.onScoringLocationClick(column, row))}
                      </Grid>
                    ))}
                    </React.Fragment>
                  ))}

                  <Grid item xs={3} sm={3} md={3}>
                    <Item>{this.state.driverStation === 1 && <AccountBoxIcon />}</Item>
                  </Grid>

                  <Grid item xs={3} sm={3} md={3}>
                    <Item>{this.state.driverStation === 2 && <AccountBoxIcon />}</Item>
                  </Grid>

                  <Grid item xs={3} sm={3} md={3}>
                    <Item>{this.state.driverStation === 3 && <AccountBoxIcon />}</Item>
                  </Grid>

                  <Grid item xs={1} sm={1} md={1}>
                    <Stack spacing={2}>
                      <Item>
                        <TextField
                          disabled
                          label="Column"
                          value={columnIndexToName(this.state.selectedColumnFromRobot)}
                        />
                      </Item>
                      <Item>
                        <TextField
                          disabled
                          label="Row"
                          value={rowIndexToName(this.state.selectedRowFromRobot)}
                        />
                      </Item>
                    </Stack>
                  </Grid>

                  <Grid item xs={1} sm={1} md={1}>
                    <Stack spacing={2}>
                      <Item>
                        <TextField
                          disabled
                          label="Shape"
                          value={this.state.selectedShapeFromRobot}
                        />
                      </Item>
                      <Item>
                        <TextField
                          disabled
                          label="Substation"
                          value={this.state.selectedSubstationFromRobot}
                        />
                      </Item>
                    </Stack>
                  </Grid>

                  <Grid item xs={2} sm={2} md={2}>
                    <Stack spacing={2} onClick={() => this.onScoringPieceClick("CONE")}>
                      {renderBigCone(this.state.selectedShapeFromRobot === "CONE")}
                    </Stack>
                  </Grid>

                  <Grid item xs={2} sm={2} md={2}>
                    <Stack spacing={2} onClick={() => this.onScoringPieceClick("CUBE")}>
                      {renderBigCube(this.state.selectedShapeFromRobot === "CUBE")}
                    </Stack>
                  </Grid>

                  <Grid item xs={1} sm={1} md={1}>
                    <Stack spacing={2}>
                      <div style={{marginBottom: darkTheme.spacing(6)}}></div>
                      {renderSubstation("SINGLE", this.state.selectedSubstationFromRobot, this.onSubstationClick.bind(this))}
                    </Stack>
                  </Grid>

                  <Grid item xs={1} sm={1} md={1}>{renderSubstation("DOUBLE_LEFT", this.state.selectedSubstationFromRobot, this.onSubstationClick.bind(this))}</Grid>
                  <Grid item xs={1} sm={1} md={1}>{renderSubstation("DOUBLE_RIGHT", this.state.selectedSubstationFromRobot, this.onSubstationClick.bind(this))}</Grid>
                </React.Fragment>
              }
            </Grid>
          </Grid>
        </Grid>
      </ThemeProvider>
    );
  }
}

export default App;
