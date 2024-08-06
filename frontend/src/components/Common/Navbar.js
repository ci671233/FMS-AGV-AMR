import React from 'react';
import { Link } from 'react-router-dom';

function Navbar() {
    return (
        <nav>
            <ul>
                <li><Link to="/main">Main</Link></li>
                <li><Link to="/monitor">Monitoring</Link></li>
                <li><Link to="/map_create">Map Create</Link></li>
                <li><Link to="/map_upload">Map Upload</Link></li>
                {/* <li><Link to="/map_edit">Map Edit</Link></li>
                <li><Link to="/robot">Robot</Link></li>
                <li><Link to="/control">Control</Link></li>
                <li><Link to="/logs">Logs</Link></li> */}
                {/* 다른 네비게이션 링크들 */}
            </ul>
        </nav>
    );
}

export default Navbar;

