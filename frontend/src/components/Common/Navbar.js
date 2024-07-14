import React from 'react';
import { Link } from 'react-router-dom';

function Navbar() {
    return (
        <nav>
            <ul>
                <li><Link to="/monitor">Monitoring</Link></li>
                <li><Link to="/map">Map Editor</Link></li>
                <li><Link to="/control">Control</Link></li>
                <li><Link to="/logs">Logs</Link></li>
                <li><Link to="/main">Main</Link></li>
                {/* 다른 네비게이션 링크들 */}
            </ul>
        </nav>
    );
}

export default Navbar;

