# Dashboard UI
This dashboard communicates with the robot over network tables using the [ntcore-ts-client](https://github.com/Chris2fourlaw/ntcore-ts-client) library.

## Setup
1. Install [Node and npm](https://nodejs.org/en/download)
2. Open a terminal and run  `npm i` to install the depedencies listed in `package.json`
3. Run `npm start` to launc the development server
4. The recommended editor is Visual Studio Code

## Preparing for Competition
React can be built into static web files, but some references don't work when opening an html file from local disk, so the files need to be `hosted`.

### Building Static Files
Run `npm run build` to compile the app, the results will be in the `build` folder.

### Hosting The Files
Run `npm install -g serve` to install the `serve` module globally, this only needs to be done once. Anytime you want to host the files in the build folder, run `serve -s build` in a terminal.

## Available Scripts

In the project directory, you can run:

### `npm start`

Runs the app in the development mode.\
Open [http://localhost:3000](http://localhost:3000) to view it in the browser.

The page will reload if you make edits.\
You will also see any lint errors in the console.

### `npm test`

Launches the test runner in the interactive watch mode.\
See the section about [running tests](https://facebook.github.io/create-react-app/docs/running-tests) for more information.

### `npm run build`

Builds the app for production to the `build` folder.\
It correctly bundles React in production mode and optimizes the build for the best performance.

The build is minified and the filenames include the hashes.\
Your app is ready to be deployed!

See the section about [deployment](https://facebook.github.io/create-react-app/docs/deployment) for more information.

### `npm run eject`

**Note: this is a one-way operation. Once you `eject`, you can’t go back!**

If you aren’t satisfied with the build tool and configuration choices, you can `eject` at any time. This command will remove the single build dependency from your project.

Instead, it will copy all the configuration files and the transitive dependencies (webpack, Babel, ESLint, etc) right into your project so you have full control over them. All of the commands except `eject` will still work, but they will point to the copied scripts so you can tweak them. At this point you’re on your own.

You don’t have to ever use `eject`. The curated feature set is suitable for small and middle deployments, and you shouldn’t feel obligated to use this feature. However we understand that this tool wouldn’t be useful if you couldn’t customize it when you are ready for it.

## Learn More

You can learn more in the [Create React App documentation](https://facebook.github.io/create-react-app/docs/getting-started).

To learn React, check out the [React documentation](https://reactjs.org/).
