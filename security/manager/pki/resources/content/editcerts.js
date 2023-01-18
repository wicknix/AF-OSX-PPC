/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

const nsIX509Cert = Components.interfaces.nsIX509Cert;
const nsX509CertDB = "@mozilla.org/security/x509certdb;1";
const nsIX509CertDB = Components.interfaces.nsIX509CertDB;
const nsIPKIParamBlock = Components.interfaces.nsIPKIParamBlock;

var certdb;
var cert;

function doPrompt(msg)
{
  let prompts = Components.classes["@mozilla.org/embedcomp/prompt-service;1"].
    getService(Components.interfaces.nsIPromptService);
  prompts.alert(window, null, msg);
}

function setWindowName()
{
  var dbkey = self.name;

  //  Get the cert from the cert database
  certdb = Components.classes[nsX509CertDB].getService(nsIX509CertDB);
  cert = certdb.findCertByDBKey(dbkey, null);

  var bundle = document.getElementById("pippki_bundle");

  var message1 = bundle.getFormattedString("editTrustCA", [cert.commonName]);
  setText("certmsg", message1);

  var ssl = document.getElementById("trustSSL");
  if (certdb.isCertTrusted(cert, nsIX509Cert.CA_CERT,
                           nsIX509CertDB.TRUSTED_SSL)) {
    ssl.setAttribute("checked", "true");
  } else {
    ssl.setAttribute("checked", "false");
  }
  var email = document.getElementById("trustEmail");
  if (certdb.isCertTrusted(cert, nsIX509Cert.CA_CERT,
                           nsIX509CertDB.TRUSTED_EMAIL)) {
    email.setAttribute("checked", "true");
  } else {
    email.setAttribute("checked", "false");
  }
  var objsign = document.getElementById("trustObjSign");
  if (certdb.isCertTrusted(cert, nsIX509Cert.CA_CERT,
                           nsIX509CertDB.TRUSTED_OBJSIGN)) {
    objsign.setAttribute("checked", "true");
  } else {
    objsign.setAttribute("checked", "false");
  }
}

function doOK()
{
  var ssl = document.getElementById("trustSSL");
  var email = document.getElementById("trustEmail");
  var objsign = document.getElementById("trustObjSign");
  var trustssl = (ssl.checked) ? nsIX509CertDB.TRUSTED_SSL : 0;
  var trustemail = (email.checked) ? nsIX509CertDB.TRUSTED_EMAIL : 0;
  var trustobjsign = (objsign.checked) ? nsIX509CertDB.TRUSTED_OBJSIGN : 0;
  //
  //  Set the cert trust
  //
  certdb.setCertTrust(cert, nsIX509Cert.CA_CERT,
                      trustssl | trustemail | trustobjsign);
  return true;
}

function doLoadForEmailCert()
{
  var dbkey = self.name;

  //  Get the cert from the cert database
  certdb = Components.classes[nsX509CertDB].getService(nsIX509CertDB);
  cert = certdb.findCertByDBKey(dbkey, null);

  var bundle = document.getElementById("pippki_bundle");

  var message1 = bundle.getFormattedString("editTrustEmail", [cert.commonName]);
  setText("certmsg", message1);

  setText("issuer", cert.issuerName);

  var cacert = getCaCertForEntityCert(cert);
  if(cacert == null)
  {
     setText("explanations", bundle.getString("issuerNotKnown"));
  }
  else if(certdb.isCertTrusted(cacert, nsIX509Cert.CA_CERT,
                               nsIX509CertDB.TRUSTED_EMAIL))
  {
     setText("explanations", bundle.getString("issuerTrusted"));
  }
  else
  {
     setText("explanations", bundle.getString("issuerNotTrusted"));
  }
  var sslTrust = document.getElementById("sslTrustGroup");
  sslTrust.value = certdb.isCertTrusted(cert, nsIX509Cert.EMAIL_CERT,
                                        nsIX509CertDB.TRUSTED_EMAIL);
}

function doEmailOK()
{
  var sslTrust = document.getElementById("sslTrustGroup");
  var trustemail = sslTrust.value == "true"
                   ? nsIX509CertDB.TRUSTED_EMAIL
                   : nsIX509CertDB.UNTRUSTED;
  //
  //  Set the cert trust
  //
  certdb.setCertTrust(cert, nsIX509Cert.EMAIL_CERT, trustemail);
  return true;
}

function editCaTrust()
{
   var cacert = getCaCertForEntityCert(cert);
   if(cacert != null)
   {
      window.openDialog('chrome://pippki/content/editcacert.xul', cacert.dbKey,
                        'chrome,centerscreen,modal');
   }
   else
   {
      var bundle = document.getElementById("pippki_bundle");
      doPrompt(bundle.getString("issuerCertNotFound"));
   }
}

function getCaCertForEntityCert(cert)
{
   var nextCertInChain;
   nextCertInChain = cert;
   var lastSubjectName="";
   while(true)
   {
     if(nextCertInChain == null)
     {
        return null;
     }
     if((nextCertInChain.type == nsIX509Cert.CA_CERT) ||
        (nextCertInChain.subjectName == lastSubjectName))
     {
        break;
     }

     lastSubjectName = nextCertInChain.subjectName;
     nextCertInChain = nextCertInChain.issuer;
   }

   return nextCertInChain;
}
