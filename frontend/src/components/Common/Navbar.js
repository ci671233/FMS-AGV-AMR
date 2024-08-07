import React from 'react';
import { Link } from 'react-router-dom';

function Navbar() {
    return (
        <nav style={{ backgroundColor: '#f8f9fa', padding: '10px 20px', borderRadius: '5px', marginBottom: '20px' }}>
            <ul style={{ listStyleType: 'none', padding: 0, margin: 0, display: 'flex', justifyContent: 'space-around' }}>
                <li style={{ margin: '0 10px' }}>
                    <Link 
                        to="/main" 
                        style={{ 
                            color: '#2c3e50', 
                            textDecoration: 'none', 
                            padding: '10px 20px', 
                            borderRadius: '5px',
                            backgroundColor: '#ecf0f1',
                            display: 'block',
                            textAlign: 'center'
                        }}
                    >
                        Main
                    </Link>
                </li>
                <li style={{ margin: '0 10px' }}>
                    <Link 
                        to="/monitor" 
                        style={{ 
                            color: '#2c3e50', 
                            textDecoration: 'none', 
                            padding: '10px 20px', 
                            borderRadius: '5px',
                            backgroundColor: '#ecf0f1',
                            display: 'block',
                            textAlign: 'center'
                        }}
                    >
                        Monitoring
                    </Link>
                </li>
                <li style={{ margin: '0 10px' }}>
                    <Link 
                        to="/map_create" 
                        style={{ 
                            color: '#2c3e50', 
                            textDecoration: 'none', 
                            padding: '10px 20px', 
                            borderRadius: '5px',
                            backgroundColor: '#ecf0f1',
                            display: 'block',
                            textAlign: 'center'
                        }}
                    >
                        Map Create
                    </Link>
                </li>
                <li style={{ margin: '0 10px' }}>
                    <Link 
                        to="/map_upload" 
                        style={{ 
                            color: '#2c3e50', 
                            textDecoration: 'none', 
                            padding: '10px 20px', 
                            borderRadius: '5px',
                            backgroundColor: '#ecf0f1',
                            display: 'block',
                            textAlign: 'center'
                        }}
                    >
                        Map Upload
                    </Link>
                </li>
                <li style={{ margin: '0 10px' }}>
                    <Link 
                        to="/map_edit" 
                        style={{ 
                            color: '#2c3e50', 
                            textDecoration: 'none', 
                            padding: '10px 20px', 
                            borderRadius: '5px',
                            backgroundColor: '#ecf0f1',
                            display: 'block',
                            textAlign: 'center'
                        }}
                    >
                        Map Edit
                    </Link>
                </li>
                <li style={{ margin: '0 10px' }}>
                    <Link 
                        to="/robot_register" 
                        style={{ 
                            color: '#2c3e50', 
                            textDecoration: 'none', 
                            padding: '10px 20px', 
                            borderRadius: '5px',
                            backgroundColor: '#ecf0f1',
                            display: 'block',
                            textAlign: 'center'
                        }}
                    >
                        Robot Register
                    </Link>
                </li>
                <li style={{ margin: '0 10px' }}>
                    <Link 
                        to="/robot_edit" 
                        style={{ 
                            color: '#2c3e50', 
                            textDecoration: 'none', 
                            padding: '10px 20px', 
                            borderRadius: '5px',
                            backgroundColor: '#ecf0f1',
                            display: 'block',
                            textAlign: 'center'
                        }}
                    >
                        Robot Edit
                    </Link>
                </li>
            </ul>
        </nav>
    );
}

export default Navbar;


