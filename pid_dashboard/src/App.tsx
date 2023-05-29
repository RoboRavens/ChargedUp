import React, { ChangeEvent } from 'react';
import raven from './raven.svg';
import './App.css';
import {NetworkTables, NetworkTablesTopic, NetworkTablesTypeInfos} from 'ntcore-ts-client';
import Oval from 'react-loading-icons/dist/esm/components/oval';

interface Topics {
  pInputPub?: NetworkTablesTopic<number>;
  iInputPub?: NetworkTablesTopic<number>;
  dInputPub?: NetworkTablesTopic<number>;
  buttonClickPub?: NetworkTablesTopic<string>;
}

const topics: Topics = {
  pInputPub: undefined,
  iInputPub: undefined,
  dInputPub: undefined,
  buttonClickPub: undefined,
}

const NT_CORE = NetworkTables.getInstanceByURI("localhost");

class App extends React.Component<{}, {
  connected: boolean,
  pInput: number,
  iInput: number,
  dInput: number,
  buttonClick: string,
}> {
  constructor(props: {}) {
    super(props);
    this.state = {
      connected: false,
      pInput: 0,
      iInput: 0,
      dInput: 0,
      buttonClick: "none"
    }
    this.handleClick = this.handleClick.bind(this);
    this.setInputState = this.setInputState.bind(this);
  }

  componentDidMount() {
    NT_CORE.addRobotConnectionListener(this.setConnected.bind(this), true);
    topics.pInputPub = NT_CORE.createTopic<number>('/ReactDash/Main/dpub/pInput', NetworkTablesTypeInfos.kDouble);
    topics.pInputPub.publish({retained: true});
    topics.iInputPub = NT_CORE.createTopic<number>('/ReactDash/Main/dpub/iInput', NetworkTablesTypeInfos.kDouble);
    topics.iInputPub.publish({retained: true});
    topics.dInputPub = NT_CORE.createTopic<number>('/ReactDash/Main/dpub/dInput', NetworkTablesTypeInfos.kDouble);
    topics.dInputPub.publish({retained: true});
  }

  setConnected(connected: boolean) {
    this.setState({
      connected: connected
    });
  }

  handleClick() {
    topics.pInputPub?.setValue(this.state.pInput);
    topics.iInputPub?.setValue(this.state.iInput);
    topics.dInputPub?.setValue(this.state.dInput);
  }

  setInputState(input: string, event: ChangeEvent<HTMLInputElement>) {
    this.setState((prevState, props) => ({
        ...prevState,
        [input]: Number(event.target.value),
    }));
  }

  render() {
    return (
      <div className="App">
        <link href="https://fonts.googleapis.com/css2?family=Material+Symbols+Outlined" rel="stylesheet" />
        <header className="App-header">
          <img src={raven} className="App-logo" alt="logo" />
          <div id="connection">{
            this.state.connected 
            ? <div><p>Connected</p><span className="material-symbols-outlined">check</span></div>
            : <div><p>Not Connected</p><Oval/></div>}</div>
          <div id="input-container">
            <input placeholder='Proportional' onChange={(event) => this.setInputState("pInput", event)}></input>
            <input placeholder='Integral' onChange={(event) => this.setInputState("iInput", event)}></input>
            <input placeholder='Derivative' onChange={(event) => this.setInputState("dInput", event)}></input>
          </div>
          <button type="submit" onClick={this.handleClick}>Update</button>
        </header>
      </div>
    );
  }
  
}

export default App;