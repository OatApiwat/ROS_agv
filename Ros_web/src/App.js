import React from 'react';
import Header from './components/Header';
import Footer from './components/Footer';
import Body from './components/Body';
import './App.css';

function App() {
  return (
    <div className="App">
      <Header />
      <div className="main-content">
        {/* <Sidebar /> */}
        <Body />
      </div>
      <Footer />
    </div>
  );
}

export default App;
