import React, { Component } from 'react';
import { Container } from 'react-bootstrap';

const footerStyle = {
  position: 'fixed',
  bottom: 0,
  width: '100%',
  backgroundColor: '#f8f9fa', // Adjust as needed
  textAlign: 'center',
  padding: '10px 0',
};

export default class Footer extends Component {
  render() {
    return (
      <footer style={footerStyle}>
        <Container>
          <p>Manufacturing Improvement Center YOKOTEN (MIC) &copy; 2024</p>
        </Container>
      </footer>
    );
  }
}
