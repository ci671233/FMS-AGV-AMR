import React, { useState, useContext } from 'react';
import axios from 'axios';
import AuthContext from '../context/AuthContext';

function ApproveAccount() {
  const [accountID, setAccountID] = useState('');
  const { token } = useContext(AuthContext);

  const handleApprove = async () => {
    try {
      const response = await axios.put(`http://172.30.1.28:8080/api/admin/approve/${accountID}`, {}, {
        headers: { Authorization: `Bearer ${token}` }
      });
      alert('Account approved successfully.');
      console.log(response.data);
    } catch (error) {
      console.error('Approval error:', error);
      alert('Approval failed.');
    }
  };

  return (
    <div>
      <h2>Approve Account</h2>
      <input type="text" placeholder="Account ID" value={accountID} onChange={(e) => setAccountID(e.target.value)} />
      <button onClick={handleApprove}>Approve Account</button>
    </div>
  );
}

export default ApproveAccount;

