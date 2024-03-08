import React from 'react';
import logo from './logo.svg';
import './App.css';
import CameraPage from './page/CameraPage';
import {
  createBrowserRouter,
  RouterProvider,
} from 'react-router-dom';
import NodeStatusPage from './page/NodeStatusPage';
import Dashboard from './page/Dashboard';


const router = createBrowserRouter([{
  path: '/',
  element: <Dashboard />,
},
{
  path: '/node/:pkg/:node',
  element: <NodeStatusPage />,
  loader: async ({ params }) => {
    const { pkg, node } = params;
    return { pkg, node };
  }
},
])

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <RouterProvider router={router} />
      </header>
    </div>
  );
}

export default App;
