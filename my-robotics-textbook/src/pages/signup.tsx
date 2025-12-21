import React, { useState } from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';

const SignupPage = () => {
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    // In a real app, this would handle registration
    console.log('Signup attempt with:', { name, email, password });
  };

  return (
    <Layout title="Sign Up" description="Create your robotics textbook account">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="padding--lg text--center">
              <h1>Create Your Account</h1>
              <p>Join the Physical AI & Humanoid Robotics learning community</p>
            </div>

            <div className="padding-horiz--md">
              <form onSubmit={handleSubmit}>
                <div className="margin-bottom--lg">
                  <label htmlFor="name" className="form-label">Full Name</label>
                  <input
                    type="text"
                    id="name"
                    className="form-control"
                    value={name}
                    onChange={(e) => setName(e.target.value)}
                    required
                  />
                </div>

                <div className="margin-bottom--lg">
                  <label htmlFor="email" className="form-label">Email</label>
                  <input
                    type="email"
                    id="email"
                    className="form-control"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    required
                  />
                </div>

                <div className="margin-bottom--lg">
                  <label htmlFor="password" className="form-label">Password</label>
                  <input
                    type="password"
                    id="password"
                    className="form-control"
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    required
                  />
                </div>

                <div className="margin-bottom--lg">
                  <button type="submit" className="button button--primary button--block">
                    Create Account
                  </button>
                </div>

                <div className="text--center">
                  <p>
                    Already have an account? <a href="/login">Sign in</a>
                  </p>
                  <p>
                    <a href="/">Back to Home</a>
                  </p>
                </div>
              </form>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default SignupPage;