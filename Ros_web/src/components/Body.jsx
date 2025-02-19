import React, { Component } from "react";
import Container from "react-bootstrap/Container";
import { BrowserRouter as Router, Route, Routes } from "react-router-dom";
import Home from "../pages/Home";
import AgvControl from "../pages/AgvControl"; // Changed to start with an uppercase letter

export default class Body extends Component {
  render() {
    return (
      <Container>
        <Router>
          <main>
            <Routes>
              <Route path="/home" element={<Home />} />
              <Route path="/agv-control" element={<AgvControl />} /> {/* Changed to use the uppercase component name */}
            </Routes>
          </main>
        </Router>
      </Container>
    );
  }
}
