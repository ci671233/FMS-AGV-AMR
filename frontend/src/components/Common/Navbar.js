import React from 'react';
import { Link } from 'react-router-dom';

function Navbar() {
    return (
        <nav>
            <ul>
                <li><Link to="/main/">Main</Link></li>
                <li><Link to="/main/monitoring">Monitoring</Link></li>
                <li><Link to="/main/control">Control</Link></li>
                <li><Link to="/main/map">Map</Link></li>
            </ul>
        </nav>
    );
}

export default Navbar;
