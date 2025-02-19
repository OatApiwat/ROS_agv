import React, { Component } from 'react';
import Container from 'react-bootstrap/Container';
import Nav from 'react-bootstrap/Nav';
import Navbar from 'react-bootstrap/Navbar';
import NavDropdown from 'react-bootstrap/NavDropdown';
// Import the logo image
import logo from '../robot-logo-removebg-preview.png'; // Adjust the path to your logo

export default class Header extends Component {
  render() {
    return (
      <Navbar bg="dark" variant="dark" expand="lg" height= '10px' collapseOnSelect>
        <Container>
          <Navbar.Brand href="/home">
            {/* Add the logo image */}
            <img
              src={logo}
              width="30"
              height="30"
              className="d-inline-block align-top"
              alt="Robot Project logo"
            />
            {' '}Robot-Project
          </Navbar.Brand>
          <Navbar.Toggle aria-controls="basic-navbar-nav" />
          <Navbar.Collapse id="basic-navbar-nav">
            <Nav className="me-auto">
              <Nav.Link href="/home">Home</Nav.Link>
              <Nav.Link href="/agv-control">AGV Project</Nav.Link>
              <NavDropdown title="Dropdown" id="basic-nav-dropdown">
                <NavDropdown.Item href="#action/3.1">Action</NavDropdown.Item>
                <NavDropdown.Item href="#action/3.2">
                  Another action
                </NavDropdown.Item>
                <NavDropdown.Item href="#action/3.3">Something</NavDropdown.Item>
                <NavDropdown.Divider />
                <NavDropdown.Item href="#action/3.4">
                  Separated link
                </NavDropdown.Item>
              </NavDropdown>
            </Nav>
          </Navbar.Collapse>
        </Container>
      </Navbar>
    );
  }
}